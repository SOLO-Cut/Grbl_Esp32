/*
  Report.cpp - Status, message and settings reporting.

  Part of Grbl
  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC
  2018 - Bart Dring (ESP32 port)

  Grbl is free software: you can redistribute it and/or modify it under the
  terms of the GNU General Public License as published by the Free Software
  Foundation, either version 3 of the License, or (at your option) any later
  version. See <http://www.gnu.org/licenses/>.

  Status report (label cutter additions are marked *):
    <Idle|MPos:0.000,0.000,0.000|Bf:15,256|FS:0,0|Pn:X|Sn:PRF|Hd:1|Fn:50,0|Seq:FEED,3|WCO:...|Ov:...>
      Pn:X    X limit switch active
    * Sn:     P = paper present, R = rear sensor covered, F = front sensor covered
    * Hd:1|2  active head
    * Fn:a,b  fan duty in percent (only when a fan is on)
    * Seq:N,s running sequence name and step
*/

#include "Grbl.h"
#include <stdarg.h>

static float wco[N_AXIS];

void grbl_send(uint8_t client, const char* text) {
    serial_write(client, text);
}

static void vsend(uint8_t client, const char* prefix, const char* suffix, const char* format, va_list arg) {
    char buf[256];
    int  n = 0;
    if (prefix) {
        n = snprintf(buf, sizeof(buf), "%s", prefix);
    }
    n += vsnprintf(buf + n, sizeof(buf) - n, format, arg);
    if (n >= (int)sizeof(buf)) {
        n = sizeof(buf) - 1;
    }
    if (suffix) {
        snprintf(buf + n, sizeof(buf) - n, "%s", suffix);
    }
    serial_write(client, buf);
}

void grbl_sendf(uint8_t client, const char* format, ...) {
    va_list arg;
    va_start(arg, format);
    vsend(client, NULL, NULL, format, arg);
    va_end(arg);
}

void grbl_msg_sendf(uint8_t client, const char* format, ...) {
    va_list arg;
    va_start(arg, format);
    vsend(client, "[MSG:", "]\r\n", format, arg);
    va_end(arg);
}

static void axis_values(const float* v, char* out, size_t len) {
    float conv = settings.report_inches ? (1.0f / MM_PER_INCH) : 1.0f;
    const char* fmt = settings.report_inches ? "%.4f" : "%.3f";
    out[0]          = '\0';
    for (int i = 0; i < N_AXIS; i++) {
        size_t used = strlen(out);
        snprintf(out + used, len - used, fmt, v[i] * conv);
        if (i < N_AXIS - 1) {
            strlcat(out, ",", len);
        }
    }
}

void report_status_message(Error status_code, uint8_t client) {
    if (status_code == Error::Ok) {
        grbl_send(client, "ok\r\n");
    } else {
        grbl_sendf(client, "error:%d\r\n", static_cast<int>(status_code));
    }
}

void report_alarm_message(ExecAlarm alarm_code) {
    grbl_sendf(CLIENT_ALL, "ALARM:%d\r\n", static_cast<int>(alarm_code));
    delay(50);
}

void report_feedback_message(Message message) {
    const char* text = "";
    switch (message) {
        case Message::CriticalEvent:
            text = "Reset to continue";
            break;
        case Message::AlarmLock:
            text = "'$H'|'$X' to unlock";
            break;
        case Message::AlarmUnlock:
            text = "Caution: Unlocked";
            break;
        case Message::Enabled:
            text = "Enabled";
            break;
        case Message::Disabled:
            text = "Disabled";
            break;
        case Message::CheckLimits:
            text = "Check limits";
            break;
        case Message::ProgramEnd:
            text = "Program End";
            break;
        case Message::RestoreDefaults:
            text = "Restoring defaults";
            break;
    }
    grbl_msg_sendf(CLIENT_ALL, "%s", text);
}

void report_init_message(uint8_t client) {
    grbl_sendf(client, "\r\nGrbl %s ['$' for help]\r\n", GRBL_VERSION);
}

void report_grbl_help(uint8_t client) {
    grbl_send(client, "[HLP:$$ $# $G $I $N $x=val $Nx=line $J=line $C $X $H $FEED $EJECT $RST=$|#|* ~ ! ? ctrl-x]\r\n");
}

void report_ngc_parameters(uint8_t client) {
    static const char* names[CoordEnd] = { "G54", "G55", "G56", "G57", "G58", "G59", "G28", "G30" };
    char               temp[64];
    for (int i = 0; i < CoordEnd; i++) {
        axis_values(coord_data_store[i], temp, sizeof(temp));
        grbl_sendf(client, "[%s:%s]\r\n", names[i], temp);
    }
    axis_values(gc_state.coord_offset, temp, sizeof(temp));
    grbl_sendf(client, "[G92:%s]\r\n", temp);
    float tlo = gc_state.tool_length_offset;
    if (settings.report_inches) {
        tlo *= INCH_PER_MM;
    }
    grbl_sendf(client, "[TLO:%.3f]\r\n", tlo);
}

void report_gcode_modes(uint8_t client) {
    char modes[96];
    const char* motion = "G1";
    switch (gc_state.modal.motion) {
        case Motion::None:
            motion = "G80";
            break;
        case Motion::Seek:
            motion = "G0";
            break;
        case Motion::Linear:
            motion = "G1";
            break;
        case Motion::CwArc:
            motion = "G2";
            break;
        case Motion::CcwArc:
            motion = "G3";
            break;
    }
    const char* plane = gc_state.modal.plane_select == Plane::XY ? "G17" : (gc_state.modal.plane_select == Plane::ZX ? "G18" : "G19");
    const char* head  = gc_state.modal.head == HeadState::Head1 ? "M3" : (gc_state.modal.head == HeadState::Head2 ? "M4" : "M5");
    snprintf(modes,
             sizeof(modes),
             "[GC:%s G%d %s %s %s %s %s T%d F%.0f S%.0f]\r\n",
             motion,
             gc_state.modal.coord_select + 54,
             plane,
             gc_state.modal.units == Units::Inches ? "G20" : "G21",
             gc_state.modal.distance == Distance::Incremental ? "G91" : "G90",
             gc_state.modal.feed_rate == FeedRate::InverseTime ? "G93" : "G94",
             head,
             gc_state.tool,
             gc_state.feed_rate,
             gc_state.head_power);
    grbl_send(client, modes);
}

void report_startup_line(uint8_t n, const char* line, uint8_t client) {
    grbl_sendf(client, "$N%d=%s\r\n", n, line);
}

void report_execute_startup_message(const char* line, Error status_code, uint8_t client) {
    grbl_sendf(client, ">%s:", line);
    report_status_message(status_code, client);
}

void report_build_info(uint8_t client) {
    grbl_sendf(client, "[VER:%s.%s:%s]\r\n", GRBL_VERSION, GRBL_VERSION_BUILD, MACHINE_NAME);
    grbl_sendf(client, "[OPT:VH,%d,%d]\r\n", BLOCK_BUFFER_SIZE - 1, RX_BUFFER_SIZE);
    grbl_sendf(client, "[MSG:Using machine:%s]\r\n", MACHINE_NAME);
}

void system_convert_array_steps_to_mpos(float* position, const volatile int32_t* steps) {
    for (int idx = 0; idx < N_AXIS; idx++) {
        position[idx] = steps[idx] / settings.steps_per_mm[idx];
    }
}

float* system_get_mpos() {
    static float position[N_AXIS];
    int32_t      snapshot[N_AXIS];
    for (int i = 0; i < N_AXIS; i++) {
        snapshot[i] = sys_position[i];
    }
    system_convert_array_steps_to_mpos(position, snapshot);
    return position;
}

void system_flag_wco_change() {
    protocol_buffer_synchronize();
    sys.report_wco_counter = 0;
}

static const char* state_text() {
    switch (sys.state) {
        case State::Idle:
            return "Idle";
        case State::Cycle:
            return "Run";
        case State::Hold:
            return sys.suspend.bit.holdComplete ? "Hold:0" : "Hold:1";
        case State::Jog:
            return "Jog";
        case State::Homing:
            return "Home";
        case State::Alarm:
            return "Alarm";
        case State::CheckMode:
            return "Check";
    }
    return "?";
}

void report_realtime_status(uint8_t client) {
    char status[256];
    char temp[64];

    // Local snapshot: this runs in the serial task, concurrently with the main task.
    float   position[N_AXIS];
    int32_t steps[N_AXIS];
    for (int i = 0; i < N_AXIS; i++) {
        steps[i] = sys_position[i];
    }
    system_convert_array_steps_to_mpos(position, steps);
    for (int i = 0; i < N_AXIS; i++) {
        wco[i] = gc_state.coord_system[i] + gc_state.coord_offset[i];
        if (i == Z_AXIS) {
            wco[i] += gc_state.tool_length_offset;
        }
    }
    bool mpos = settings.status_mask & bit(0);
    if (!mpos) {
        for (int i = 0; i < N_AXIS; i++) {
            position[i] -= wco[i];
        }
    }
    axis_values(position, temp, sizeof(temp));
    snprintf(status, sizeof(status), "<%s|%s:%s", state_text(), mpos ? "MPos" : "WPos", temp);

    if (settings.status_mask & bit(1)) {
        snprintf(temp, sizeof(temp), "|Bf:%d,%d", plan_get_block_buffer_available(), serial_rx_available(client == CLIENT_ALL ? CLIENT_CONSOLE : client));
        strlcat(status, temp, sizeof(status));
    }
    plan_block_t* cur = plan_get_current_block();
    if (cur != NULL && cur->line_number > 0) {
        snprintf(temp, sizeof(temp), "|Ln:%d", (int)cur->line_number);
        strlcat(status, temp, sizeof(status));
    }
    float rate  = st_get_realtime_rate();
    HeadState h = heads_get_state();
    snprintf(temp,
             sizeof(temp),
             "|FS:%.0f,%.0f",
             settings.report_inches ? rate / MM_PER_INCH : rate,
             h == HeadState::Off ? 0.0f : gc_state.head_power);
    strlcat(status, temp, sizeof(status));

    if (limits_x_triggered()) {
        strlcat(status, "|Pn:X", sizeof(status));
    }
    uint8_t s = machine_sensors();
    if (s) {
        snprintf(temp,
                 sizeof(temp),
                 "|Sn:%s%s%s",
                 (s & SENSOR_PAPER) ? "P" : "",
                 (s & SENSOR_REAR) ? "R" : "",
                 (s & SENSOR_FRONT) ? "F" : "");
        strlcat(status, temp, sizeof(status));
    }
    if (h != HeadState::Off) {
        snprintf(temp, sizeof(temp), "|Hd:%d", h == HeadState::Head1 ? 1 : 2);
        strlcat(status, temp, sizeof(status));
    }
    if (fans_get(0) > 0 || fans_get(1) > 0) {
        snprintf(temp, sizeof(temp), "|Fn:%.0f,%.0f", fans_get(0), fans_get(1));
        strlcat(status, temp, sizeof(status));
    }
    const char* seq = machine_sequence_name();
    if (seq) {
        snprintf(temp, sizeof(temp), "|Seq:%s,%d", seq, machine_sequence_step());
        strlcat(status, temp, sizeof(status));
    }
    if (sys.report_wco_counter > 0) {
        sys.report_wco_counter--;
    } else {
        sys.report_wco_counter = (sys.state == State::Idle || sys.state == State::Alarm) ? 9 : 29;
        if (sys.report_ovr_counter == 0) {
            sys.report_ovr_counter = 1;
        }
        axis_values(wco, temp, sizeof(temp));
        strlcat(status, "|WCO:", sizeof(status));
        strlcat(status, temp, sizeof(status));
    }
    if (sys.report_ovr_counter > 0) {
        sys.report_ovr_counter--;
    } else {
        sys.report_ovr_counter = (sys.state == State::Idle || sys.state == State::Alarm) ? 9 : 19;
        snprintf(temp, sizeof(temp), "|Ov:%d,%d,100", sys.f_override, sys.r_override);
        strlcat(status, temp, sizeof(status));
    }
    strlcat(status, ">\r\n", sizeof(status));
    grbl_send(client, status);
}
