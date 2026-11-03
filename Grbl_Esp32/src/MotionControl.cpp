/*
  MotionControl.cpp - High level motion interface, homing, limits and jogging.

  Part of Grbl
  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  2018 - Bart Dring (ESP32 port)

  Grbl is free software: you can redistribute it and/or modify it under the
  terms of the GNU General Public License as published by the Free Software
  Foundation, either version 3 of the License, or (at your option) any later
  version. See <http://www.gnu.org/licenses/>.

  Label cutter specifics:
  * Only X has a limit switch. Homing ($H) homes X only.
  * Soft limits are applied to X only: 0 <= X <= $130. Y and Z are unbounded.
  * The X limit switch is handled by a GPIO interrupt. During homing the ISR
    locks the X axis on the switch edge; otherwise it raises a hard-limit alarm.
*/

#include "Grbl.h"
#include <driver/gpio.h>
#include <soc/gpio_reg.h>

static_assert(X_LIMIT_PIN < 32, "X limit must be GPIO0..31");

#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif

static volatile bool homing_approach = false;  // ISR: lock X on the first switch edge

// Execute linear motion in absolute mm coordinates. Primary gateway to the planner.
bool mc_line(float* target, plan_line_data_t* pl_data) {
    bool submitted       = false;
    sys_pl_data_inflight = pl_data;
    if (sys.state == State::CheckMode) {
        sys_pl_data_inflight = NULL;
        return false;
    }
    // Wait for room in the planner buffer.
    while (true) {
        protocol_execute_realtime();
        if (sys.abort) {
            sys_pl_data_inflight = NULL;
            return false;
        }
        if (plan_check_full_buffer()) {
            protocol_auto_cycle_start();
            protocol_wait(1);
        } else {
            break;
        }
    }
    if (sys_pl_data_inflight == pl_data) {  // Not cancelled by a jog cancel in the meantime
        plan_buffer_line(target, pl_data);
        submitted = true;
    }
    sys_pl_data_inflight = NULL;
    return submitted;
}

// Arc in offset mode. Approximated by short line segments within $12 tolerance.
void mc_arc(float*            target,
            plan_line_data_t* pl_data,
            float*            position,
            float*            offset,
            float             radius,
            uint8_t           axis_0,
            uint8_t           axis_1,
            uint8_t           axis_linear,
            bool              is_clockwise_arc) {
    float center_axis0 = position[axis_0] + offset[axis_0];
    float center_axis1 = position[axis_1] + offset[axis_1];
    float r_axis0      = -offset[axis_0];
    float r_axis1      = -offset[axis_1];
    float rt_axis0     = target[axis_0] - center_axis0;
    float rt_axis1     = target[axis_1] - center_axis1;

    float angular_travel = atan2f(r_axis0 * rt_axis1 - r_axis1 * rt_axis0, r_axis0 * rt_axis0 + r_axis1 * rt_axis1);
    if (is_clockwise_arc) {
        if (angular_travel >= -ARC_ANGULAR_TRAVEL_EPSILON) {
            angular_travel -= 2 * M_PI;
        }
    } else {
        if (angular_travel <= ARC_ANGULAR_TRAVEL_EPSILON) {
            angular_travel += 2 * M_PI;
        }
    }
    float    tol      = settings.arc_tolerance;
    uint16_t segments = floorf(fabsf(0.5f * angular_travel * radius) / sqrtf(tol * (2 * radius - tol)));
    if (segments) {
        if (pl_data->motion.inverseTime) {
            pl_data->feed_rate *= segments;
            pl_data->motion.inverseTime = 0;
        }
        float theta_per_segment  = angular_travel / segments;
        float linear_per_segment = (target[axis_linear] - position[axis_linear]) / segments;
        float cos_T              = 2.0f - theta_per_segment * theta_per_segment;
        float sin_T              = theta_per_segment * 0.16666667f * (cos_T + 4.0f);
        cos_T *= 0.5f;
        uint8_t count = 0;
        for (uint16_t i = 1; i < segments; i++) {
            if (count < N_ARC_CORRECTION) {
                float r_axisi = r_axis0 * sin_T + r_axis1 * cos_T;
                r_axis0       = r_axis0 * cos_T - r_axis1 * sin_T;
                r_axis1       = r_axisi;
                count++;
            } else {
                float cos_Ti = cosf(i * theta_per_segment);
                float sin_Ti = sinf(i * theta_per_segment);
                r_axis0      = -offset[axis_0] * cos_Ti + offset[axis_1] * sin_Ti;
                r_axis1      = -offset[axis_0] * sin_Ti - offset[axis_1] * cos_Ti;
                count        = 0;
            }
            position[axis_0] = center_axis0 + r_axis0;
            position[axis_1] = center_axis1 + r_axis1;
            position[axis_linear] += linear_per_segment;
            limits_check_soft(position);
            mc_line(position, pl_data);
            if (sys.abort) {
                return;
            }
        }
    }
    limits_check_soft(target);
    mc_line(target, pl_data);
}

bool delay_msec(int32_t milliseconds) {
    int64_t end = esp_timer_get_time() + (int64_t)milliseconds * 1000;
    while (esp_timer_get_time() < end) {
        if (sys.abort) {
            return false;
        }
        protocol_execute_realtime();
        int32_t left_ms = (int32_t)((end - esp_timer_get_time()) / 1000);
        if (left_ms > 0) {
            protocol_wait(pdMS_TO_TICKS(MIN(left_ms, DWELL_TIME_STEP)) + 1);
        }
    }
    return true;
}

bool mc_dwell(int32_t milliseconds) {
    if (milliseconds <= 0 || sys.state == State::CheckMode) {
        return false;
    }
    protocol_buffer_synchronize();
    return delay_msec(milliseconds);
}

// ---------------------------------------------------------------------------
// Limits
// ---------------------------------------------------------------------------
bool IRAM_ATTR limits_x_triggered() {
    bool level = (REG_READ(GPIO_IN_REG) >> X_LIMIT_PIN) & 1;
    return settings.limit_invert ? level : !level;  // Default: active LOW (switch to GND)
}

static void IRAM_ATTR isr_x_limit(void* arg) {
    if (!limits_x_triggered()) {
        return;  // Release edge
    }
    if (sys.state == State::Homing) {
        if (homing_approach) {
            sys.homing_axis_lock &= ~bit(X_AXIS);  // Stop X right at the switch edge
            protocol_notify_from_isr();
        }
        return;
    }
    if (settings.hard_limits && sys.state != State::Alarm && sys_rt_exec_alarm == ExecAlarm::None) {
        mc_reset();
        sys_rt_exec_alarm = ExecAlarm::HardLimit;
        protocol_notify_from_isr();
    }
}

void limits_init() {
    static bool installed = false;
    if (!installed) {
        installed = true;
        pinMode(X_LIMIT_PIN, INPUT_PULLUP);
        gpio_set_intr_type((gpio_num_t)X_LIMIT_PIN, GPIO_INTR_ANYEDGE);
        gpio_isr_handler_add((gpio_num_t)X_LIMIT_PIN, isr_x_limit, NULL);
        gpio_intr_enable((gpio_num_t)X_LIMIT_PIN);
    }
}

// Soft limit range for X: 0 .. $130. Y and Z (sheet transport, feed roller) are unbounded.
bool limits_check_travel(const float* target) {
    if (settings.x_max_travel > 0) {
        if (target[X_AXIS] < 0.0f || target[X_AXIS] > settings.x_max_travel) {
            return true;
        }
    }
    return false;
}

void limits_check_soft(float* target) {
    if (!settings.soft_limits || sys.state == State::Jog || sys.state == State::Homing) {
        return;
    }
    if (limits_check_travel(target)) {
        sys.soft_limit = true;
        if (sys.state == State::Cycle) {
            rt_set(ExecFlag::FeedHold);
            do {
                protocol_execute_realtime();
                if (sys.abort) {
                    return;
                }
                protocol_wait(1);
            } while (sys.state != State::Idle);
        }
        mc_reset();
        sys_rt_exec_alarm = ExecAlarm::SoftLimit;
        protocol_execute_realtime();
    }
}

// ---------------------------------------------------------------------------
// Homing (X only, event driven: the limit ISR locks the axis on the switch edge)
// ---------------------------------------------------------------------------
static bool homing_move(float distance, float rate, bool approach) {
    plan_line_data_t pl_data = {};
    pl_data.motion.systemMotion   = 1;
    pl_data.motion.noFeedOverride = 1;
    pl_data.feed_rate             = rate;

    float target[N_AXIS];
    memcpy(target, system_get_mpos(), sizeof(target));
    sys_position[X_AXIS] = 0;
    target[X_AXIS]       = distance;

    homing_approach      = approach;
    sys.homing_axis_lock = bit(X_AXIS);
    if (approach && limits_x_triggered()) {
        sys.homing_axis_lock = 0;  // Already on the switch
    }
    plan_buffer_line(target, &pl_data);
    sys.step_control                  = {};
    sys.step_control.executeSysMotion = true;
    st_prep_buffer();
    st_wake_up();

    while (true) {
        st_prep_buffer();
        if (rt_is(ExecFlag::Reset)) {
            sys_rt_exec_alarm = ExecAlarm::HomingFailReset;
            return false;
        }
        if (approach && !(sys.homing_axis_lock & bit(X_AXIS))) {
            break;  // Switch found (set by the limit ISR)
        }
        if (cycle_stop) {
            cycle_stop = false;
            if (approach) {
                sys_rt_exec_alarm = ExecAlarm::HomingFailApproach;
                return false;
            }
            if (limits_x_triggered()) {
                sys_rt_exec_alarm = ExecAlarm::HomingFailPulloff;
                return false;
            }
            break;  // Pull-off complete
        }
        protocol_wait(1);
    }
    homing_approach = false;
    st_reset();
    delay(settings.homing_debounce);
    return true;
}

void mc_homing_cycle() {
    if (sys.abort) {
        return;
    }
    State saved = sys.state;
    sys.state   = State::Homing;
    heads_set(HeadState::Off, 0);

    // Direction toward the switch: $23 bit0 set -> switch at the negative end.
    bool  negative   = settings.homing_dir_mask & bit(X_AXIS);
    float dir        = negative ? -1.0f : 1.0f;
    float max_travel = HOMING_AXIS_SEARCH_SCALAR * (settings.x_max_travel > 0 ? settings.x_max_travel : 1000.0f);
    float pulloff    = settings.homing_pulloff;
    bool  ok         = homing_move(dir * max_travel, settings.homing_seek, true);
    for (int i = 0; ok && i < HOMING_LOCATE_CYCLES; i++) {
        ok = homing_move(-dir * pulloff, settings.homing_seek, false);
        ok = ok && homing_move(dir * pulloff * HOMING_AXIS_LOCATE_SCALAR, settings.homing_feed, true);
    }
    ok = ok && homing_move(-dir * pulloff, settings.homing_seek, false);
    sys.step_control = {};

    if (!ok) {
        homing_approach = false;
        mc_reset();
        protocol_execute_realtime();
        return;
    }
    // Machine X: 0 at a negative-end switch, $130 at a positive-end switch, minus the pull-off.
    float home_x         = negative ? pulloff : (settings.x_max_travel - pulloff);
    sys_position[X_AXIS] = lroundf(home_x * settings.steps_per_mm[X_AXIS]);
    sys.state            = (saved == State::Alarm) ? State::Idle : saved;
    gc_sync_position();
    plan_sync_position();
}

// ---------------------------------------------------------------------------
// Reset: callable from task or ISR context. Stops the heads and steppers.
// ---------------------------------------------------------------------------
void IRAM_ATTR mc_reset() {
    if (!rt_is(ExecFlag::Reset)) {
        rt_set(ExecFlag::Reset);
        heads_kill();
        if (sys.state == State::Cycle || sys.state == State::Homing || sys.state == State::Jog || sys.step_control.executeHold ||
            sys.step_control.executeSysMotion) {
            if (sys.state == State::Homing) {
                if (sys_rt_exec_alarm == ExecAlarm::None) {
                    sys_rt_exec_alarm = ExecAlarm::HomingFailReset;
                }
            } else {
                sys_rt_exec_alarm = ExecAlarm::AbortCycle;
            }
            st_go_idle();  // Kill steppers: position is likely lost
        }
        protocol_notify_from_isr();
    }
}

// ---------------------------------------------------------------------------
// Jogging ($J=)
// ---------------------------------------------------------------------------
Error jog_execute(plan_line_data_t* pl_data, parser_block_t* gc_block, bool* cancelledInflight) {
    pl_data->feed_rate             = gc_block->values.f;
    pl_data->motion.noFeedOverride = 1;
    pl_data->is_jog                = true;
    pl_data->line_number           = gc_block->values.n;
    if (settings.soft_limits && limits_check_travel(gc_block->values.xyz)) {
        return Error::TravelExceeded;
    }
    if (!mc_line(gc_block->values.xyz, pl_data)) {
        *cancelledInflight = true;
        return Error::JogCancelled;
    }
    if (sys.state == State::Idle && plan_get_current_block() != NULL) {
        sys.state = State::Jog;
        st_prep_buffer();
        st_wake_up();
    }
    return Error::Ok;
}
