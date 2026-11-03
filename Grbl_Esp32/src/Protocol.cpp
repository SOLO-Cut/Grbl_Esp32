/*
  Protocol.cpp - Main loop, line execution and the realtime state machine.

  Part of Grbl
  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  2018 - Bart Dring (ESP32 port)

  Grbl is free software: you can redistribute it and/or modify it under the
  terms of the GNU General Public License as published by the Free Software
  Foundation, either version 3 of the License, or (at your option) any later
  version. See <http://www.gnu.org/licenses/>.

  Event driven main loop
  ----------------------
  The main task sleeps on a FreeRTOS task notification whenever it has nothing
  to do. It is woken by:
    * the serial RX task (new characters / realtime commands),
    * the step ISR (a segment was consumed -> refill the segment buffer),
    * the sensor and limit ISRs (sequence events, interlocks, alarms).
  A short timeout is kept only as a safety net.
*/

#include "Grbl.h"

static TaskHandle_t main_task = NULL;

typedef struct {
    char buffer[LINE_BUFFER_SIZE];
    int  len;
} client_line_t;
static client_line_t client_lines[CLIENT_COUNT];

void protocol_notify() {
    if (main_task) {
        xTaskNotifyGive(main_task);
    }
}

void IRAM_ATTR protocol_notify_from_isr() {
    if (!main_task) {
        return;
    }
    if (xPortInIsrContext()) {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(main_task, &woken);
        if (woken) {
            portYIELD_FROM_ISR();
        }
    } else {
        xTaskNotifyGive(main_task);
    }
}

void protocol_wait(uint32_t ticks) {
    ulTaskNotifyTake(pdTRUE, ticks);
}

static void empty_line(uint8_t client) {
    client_lines[client].len       = 0;
    client_lines[client].buffer[0] = '\0';
}

static Error add_char_to_line(char c, uint8_t client) {
    client_line_t* cl = &client_lines[client];
    if (c == '\b') {
        if (cl->len) {
            cl->buffer[--cl->len] = '\0';
        }
        return Error::Ok;
    }
    if (c == '\r' || c == '\n') {
        cl->len = 0;
        return Error::Eol;
    }
    if (cl->len == (LINE_BUFFER_SIZE - 1)) {
        return Error::Overflow;
    }
    cl->buffer[cl->len++] = c;
    cl->buffer[cl->len]   = '\0';
    return Error::Ok;
}

static Error execute_line(char* line, uint8_t client) {
    if (line[0] == 0) {
        return Error::Ok;
    }
    if (line[0] == '$') {
        return system_execute_line(line, client);
    }
    if (sys.state == State::Alarm || sys.state == State::Jog) {
        return Error::SystemGcLock;
    }
    return gc_execute_line(line, client);
}

void protocol_main_loop() {
    if (!main_task) {
        main_task = xTaskGetCurrentTaskHandle();
    }
    serial_reset_read_buffer(CLIENT_ALL);
    for (uint8_t c = 0; c < CLIENT_COUNT; c++) {
        empty_line(c);
    }
    if (settings.hard_limits && limits_x_triggered()) {
        sys.state = State::Alarm;
        report_feedback_message(Message::CheckLimits);
    }
    if (sys.state == State::Alarm) {
        report_feedback_message(Message::AlarmLock);
    } else {
        sys.state = State::Idle;
        char line[LINE_BUFFER_SIZE];
        system_execute_startup(line);
    }

    for (;;) {
        bool busy = false;
        for (uint8_t client = 0; client < CLIENT_COUNT; client++) {
            if (serial_take_overflow(client)) {
                report_status_message(Error::Overflow, client);
                empty_line(client);
            }
            int c;
            while ((c = serial_read(client)) != -1) {
                busy = true;
                Error res = add_char_to_line(c, client);
                if (res == Error::Eol) {
                    protocol_execute_realtime();
                    if (sys.abort) {
                        return;
                    }
                    report_status_message(execute_line(client_lines[client].buffer, client), client);
                    empty_line(client);
                    if (sys.abort) {
                        return;
                    }
                } else if (res == Error::Overflow) {
                    report_status_message(Error::Overflow, client);
                    empty_line(client);
                }
            }
        }
        protocol_auto_cycle_start();
        protocol_execute_realtime();
        if (sys.abort) {
            return;
        }
        st_check_idle_disable();
        if (!busy) {
            protocol_wait(pdMS_TO_TICKS(10));
        }
    }
}

// Block until all buffered motion has executed.
void protocol_buffer_synchronize() {
    protocol_auto_cycle_start();
    do {
        protocol_execute_realtime();
        if (sys.abort) {
            return;
        }
        if (plan_get_current_block() || (sys.state == State::Cycle)) {
            protocol_wait(1);
        }
    } while (plan_get_current_block() || (sys.state == State::Cycle));
}

void protocol_auto_cycle_start() {
    if (plan_get_current_block() != NULL) {
        rt_set(ExecFlag::CycleStart);
    }
}

// Executes run-time commands. Called from every wait point of the main task.
void protocol_execute_realtime() {
    protocol_exec_rt_system();
    // Hold: stay here until resumed (~), cancelled or reset.
    while (sys.suspend.value) {
        if (sys.abort) {
            return;
        }
        protocol_exec_rt_system();
        if (sys.suspend.value) {
            protocol_wait(1);
        }
    }
}

void protocol_exec_rt_system() {
    ExecAlarm alarm = sys_rt_exec_alarm;
    if (alarm != ExecAlarm::None) {
        sys.state = State::Alarm;
        report_alarm_message(alarm);
        if (alarm == ExecAlarm::HardLimit || alarm == ExecAlarm::SoftLimit) {
            // Critical event: block everything except status reports until reset.
            report_feedback_message(Message::CriticalEvent);
            rt_clear(ExecFlag::Reset);
            while (!rt_is(ExecFlag::Reset)) {
                protocol_wait(pdMS_TO_TICKS(10));
            }
        }
        sys_rt_exec_alarm = ExecAlarm::None;
    }

    machine_poll();

    uint32_t rt = sys_rt_exec_state.load();
    if (rt || cycle_stop) {
        if (rt & ExecFlag::Reset) {
            sys.abort = true;
            return;
        }
        if (rt & ExecFlag::StatusReport) {
            rt_clear(ExecFlag::StatusReport);
            report_realtime_status(CLIENT_ALL);
        }
        if (rt & (ExecFlag::MotionCancel | ExecFlag::FeedHold)) {
            if (!(sys.state == State::Alarm || sys.state == State::CheckMode)) {
                if (sys.state == State::Cycle || sys.state == State::Jog) {
                    if (!(sys.suspend.bit.motionCancel || sys.suspend.bit.jogCancel)) {
                        st_update_plan_block_parameters();
                        sys.step_control             = {};
                        sys.step_control.executeHold = true;
                        if (sys.state == State::Jog) {
                            sys.suspend.bit.jogCancel = true;
                        }
                    }
                }
                if (sys.state == State::Idle) {
                    sys.suspend.value = 0;
                    if (rt & ExecFlag::FeedHold) {
                        sys.suspend.bit.holdComplete = true;
                    }
                }
                if (rt & ExecFlag::MotionCancel) {
                    if (sys.state != State::Jog && sys.state != State::Idle) {
                        sys.suspend.bit.motionCancel = true;
                    }
                }
                if (rt & ExecFlag::FeedHold) {
                    if (sys.state != State::Jog) {
                        sys.state = State::Hold;
                    }
                }
            }
            rt_clear(ExecFlag::MotionCancel | ExecFlag::FeedHold);
        }
        if (rt & ExecFlag::CycleStart) {
            if (!(rt & (ExecFlag::FeedHold | ExecFlag::MotionCancel))) {
                if (sys.state == State::Idle || (sys.state == State::Hold && sys.suspend.bit.holdComplete)) {
                    if (sys.state == State::Hold && !machine_feed_resume_allowed()) {
                        grbl_msg_sendf(CLIENT_ALL, "No paper - load paper to resume");
                    } else {
                        sys.step_control = {};
                        if (plan_get_current_block() && !sys.suspend.bit.motionCancel) {
                            sys.suspend.value = 0;
                            sys.state         = State::Cycle;
                            st_prep_buffer();
                            st_wake_up();
                        } else {
                            sys.suspend.value = 0;
                            sys.state         = State::Idle;
                        }
                    }
                }
            }
            rt_clear(ExecFlag::CycleStart);
        }
        if (cycle_stop) {
            if (sys.state == State::Hold && !sys.soft_limit && !sys.suspend.bit.jogCancel) {
                // Hold complete; remain in Hold until resumed.
                plan_cycle_reinitialize();
                if (sys.step_control.executeHold) {
                    sys.suspend.bit.holdComplete = true;
                }
                sys.step_control.executeHold      = false;
                sys.step_control.executeSysMotion = false;
            } else {
                // Motion complete (cycle, jog, motion cancel, soft limit).
                if (sys.suspend.bit.jogCancel) {
                    sys.step_control = {};
                    plan_reset();
                    st_reset();
                    gc_sync_position();
                    plan_sync_position();
                }
                sys.suspend.value = 0;
                if (sys.state != State::Homing) {
                    sys.state = State::Idle;
                }
            }
            cycle_stop = false;
        }
    }

    if ((sys_rt_f_override != sys.f_override) || (sys_rt_r_override != sys.r_override)) {
        sys.f_override         = sys_rt_f_override;
        sys.r_override         = sys_rt_r_override;
        sys.report_ovr_counter = 0;
        plan_update_velocity_profile_parameters();
        plan_cycle_reinitialize();
    }

    switch (sys.state) {
        case State::Cycle:
        case State::Hold:
        case State::Homing:
        case State::Jog:
            st_prep_buffer();
            break;
        default:
            break;
    }
}
