/*
  Settings.cpp - $-settings, non-volatile storage and $ system commands.

  Part of Grbl
  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  2018 - Bart Dring (ESP32 port)

  Grbl is free software: you can redistribute it and/or modify it under the
  terms of the GNU General Public License as published by the Free Software
  Foundation, either version 3 of the License, or (at your option) any later
  version. See <http://www.gnu.org/licenses/>.

  Settings, coordinate systems and startup lines are kept in NVS (namespace
  "grbl") using the Preferences library. The settings block carries a
  version number; when the layout changes the defaults are restored.
*/

#include "Grbl.h"
#include <Preferences.h>

static const uint32_t SETTINGS_VERSION = 0x4C430001;  // "LC" v1

settings_t settings;
float      coord_data_store[CoordEnd][N_AXIS];
static char startup_lines[N_STARTUP_LINE][LINE_BUFFER_SIZE];
static Preferences prefs;

// ---------------------------------------------------------------------------
// Defaults
// ---------------------------------------------------------------------------
static const settings_t defaults = {
    /* $0   */ 4,
    /* $1   */ 250,
    /* $2   */ 0,
    /* $3   */ 0,
    /* $4   */ 0,
    /* $5   */ 0,
    /* $10  */ 1,
    /* $11  */ 0.010f,
    /* $12  */ 0.002f,
    /* $13  */ 0,
    /* $20  */ 1,
    /* $21  */ 1,
    /* $22  */ 1,
    /* $23  */ 1,  // X switch at X = 0
    /* $24  */ 200.0f,
    /* $25  */ 2000.0f,
    /* $26  */ 250,
    /* $27  */ 2.0f,
    /* $30  */ 1000.0f,
    /* $100-$102 */ { 80.0f, 80.0f, 80.0f },
    /* $110-$112 */ { 12000.0f, 12000.0f, 6000.0f },
    /* $120-$122 */ { 1500.0f, 1500.0f, 800.0f },
    /* $130 */ 350.0f,
    /* $300 */ 20.0f,
    /* $301 */ 10.0f,
    /* $302 */ 3000.0f,
    /* $303 */ 600.0f,
    /* $304 */ 0.0f,
    /* $305 */ 0,
    /* $306 */ 1,
    /* $307 */ 0,
    /* $308 */ 0,
    /* $309 */ 0,
};

// ---------------------------------------------------------------------------
// Setting descriptor table
// ---------------------------------------------------------------------------
enum class SType : uint8_t { U8, U16, Bool, Mask, Float };
struct SettingDesc {
    uint16_t    number;
    SType       type;
    void*       ptr;
    float       min;
    float       max;
};

#define S(num, type, field, mn, mx) { num, SType::type, (void*)&settings.field, mn, mx }
static const SettingDesc table[] = {
    S(0, U8, pulse_us, 1, 100),
    S(1, U8, idle_lock_ms, 0, 255),
    S(2, Mask, step_invert, 0, 7),
    S(3, Mask, dir_invert, 0, 7),
    S(4, Bool, enable_invert, 0, 1),
    S(5, Bool, limit_invert, 0, 1),
    S(10, U8, status_mask, 0, 3),
    S(11, Float, junction_dev, 0, 10),
    S(12, Float, arc_tolerance, 0.0001f, 1),
    S(13, Bool, report_inches, 0, 1),
    S(20, Bool, soft_limits, 0, 1),
    S(21, Bool, hard_limits, 0, 1),
    S(22, Bool, homing_enable, 0, 1),
    S(23, Mask, homing_dir_mask, 0, 7),
    S(24, Float, homing_feed, 1, 100000),
    S(25, Float, homing_seek, 1, 100000),
    S(26, U16, homing_debounce, 0, 10000),
    S(27, Float, homing_pulloff, 0, 1000),
    S(30, Float, head_s_max, 1, 100000),
    S(100, Float, steps_per_mm[0], 0.001f, 100000),
    S(101, Float, steps_per_mm[1], 0.001f, 100000),
    S(102, Float, steps_per_mm[2], 0.001f, 100000),
    S(110, Float, max_rate[0], 1, 1000000),
    S(111, Float, max_rate[1], 1, 1000000),
    S(112, Float, max_rate[2], 1, 1000000),
    S(120, Float, acceleration[0], 1, 100000),
    S(121, Float, acceleration[1], 1, 100000),
    S(122, Float, acceleration[2], 1, 100000),
    S(130, Float, x_max_travel, 0, 100000),
    S(300, Float, feed_dr, 0, 10000),
    S(301, Float, feed_df, -10000, 10000),
    S(302, Float, seq_speed, 1, 1000000),
    S(303, Float, seq_max_search, 1, 100000),
    S(304, Float, eject_extra, 0, 10000),
    S(305, Mask, sensor_invert, 0, 7),
    S(306, Bool, zero_after_feed, 0, 1),
    S(307, U16, head_pullin_ms, 0, 10000),
    S(308, U16, head_switch_ms, 0, 10000),
    S(309, U8, dir_delay_us, 0, 100),
};
#undef S

static const SettingDesc* find_setting(int number) {
    for (const auto& d : table) {
        if (d.number == number) {
            return &d;
        }
    }
    return NULL;
}

static void format_setting(const SettingDesc& d, char* out, size_t len) {
    switch (d.type) {
        case SType::U8:
        case SType::Bool:
        case SType::Mask:
            snprintf(out, len, "$%d=%d\r\n", d.number, *(uint8_t*)d.ptr);
            break;
        case SType::U16:
            snprintf(out, len, "$%d=%d\r\n", d.number, *(uint16_t*)d.ptr);
            break;
        case SType::Float:
            snprintf(out, len, "$%d=%.3f\r\n", d.number, *(float*)d.ptr);
            break;
    }
}

void report_grbl_settings(uint8_t client) {
    char line[48];
    for (const auto& d : table) {
        format_setting(d, line, sizeof(line));
        grbl_send(client, line);
    }
}

// ---------------------------------------------------------------------------
// NVS
// ---------------------------------------------------------------------------
static Error write_settings() {
    protocol_buffer_synchronize();  // Flash writes stall the cache; never during motion
    size_t n = prefs.putBytes("settings", &settings, sizeof(settings));
    prefs.putUInt("version", SETTINGS_VERSION);
    return n == sizeof(settings) ? Error::Ok : Error::NvsSetFailed;
}

static Error write_coords() {
    protocol_buffer_synchronize();
    size_t n = prefs.putBytes("coords", coord_data_store, sizeof(coord_data_store));
    return n == sizeof(coord_data_store) ? Error::Ok : Error::NvsSetFailed;
}

Error settings_store_coord(uint8_t idx, const float* data) {
    memcpy(coord_data_store[idx], data, sizeof(float) * N_AXIS);
    return write_coords();
}

void settings_read_coord(uint8_t idx, float* data) {
    memcpy(data, coord_data_store[idx], sizeof(float) * N_AXIS);
}

static void settings_restore(char which) {
    if (which == '$' || which == '*') {
        settings = defaults;
        write_settings();
    }
    if (which == '#' || which == '*') {
        memset(coord_data_store, 0, sizeof(coord_data_store));
        write_coords();
    }
    if (which == '*') {
        for (int i = 0; i < N_STARTUP_LINE; i++) {
            startup_lines[i][0] = '\0';
            char key[4]         = { 'N', char('0' + i), 0 };
            prefs.putString(key, "");
        }
    }
}

void settings_init() {
    prefs.begin("grbl", false);
    bool ok = prefs.getUInt("version", 0) == SETTINGS_VERSION && prefs.getBytesLength("settings") == sizeof(settings) &&
              prefs.getBytes("settings", &settings, sizeof(settings)) == sizeof(settings);
    if (!ok) {
        settings = defaults;
        prefs.putBytes("settings", &settings, sizeof(settings));
        prefs.putUInt("version", SETTINGS_VERSION);
        grbl_msg_sendf(CLIENT_ALL, "Settings restored to defaults");
    }
    if (prefs.getBytes("coords", coord_data_store, sizeof(coord_data_store)) != sizeof(coord_data_store)) {
        memset(coord_data_store, 0, sizeof(coord_data_store));
    }
    for (int i = 0; i < N_STARTUP_LINE; i++) {
        char key[4] = { 'N', char('0' + i), 0 };
        String s    = prefs.getString(key, "");
        strlcpy(startup_lines[i], s.c_str(), LINE_BUFFER_SIZE);
    }
}

static Error set_setting(int number, char* value) {
    const SettingDesc* d = find_setting(number);
    if (!d) {
        return Error::InvalidStatement;
    }
    uint8_t pos = 0;
    float   v;
    if (!read_float(value, &pos, &v) || value[pos] != '\0') {
        return Error::BadNumberFormat;
    }
    if (v < d->min || v > d->max) {
        return Error::NumberRange;
    }
    if (number == 0 && v < 2) {
        return Error::SettingStepPulseMin;
    }
    if (number == 20 && v != 0 && !settings.homing_enable) {
        return Error::SoftLimitError;  // Soft limits need a homed X
    }
    switch (d->type) {
        case SType::U8:
        case SType::Bool:
        case SType::Mask:
            *(uint8_t*)d->ptr = (uint8_t)v;
            break;
        case SType::U16:
            *(uint16_t*)d->ptr = (uint16_t)v;
            break;
        case SType::Float:
            *(float*)d->ptr = v;
            break;
    }
    if (number == 22 && !settings.homing_enable) {
        settings.soft_limits = 0;
    }
    return write_settings();
}

// ---------------------------------------------------------------------------
// Startup lines
// ---------------------------------------------------------------------------
void system_execute_startup(char* line) {
    for (int n = 0; n < N_STARTUP_LINE; n++) {
        if (startup_lines[n][0]) {
            strlcpy(line, startup_lines[n], LINE_BUFFER_SIZE);
            Error status = gc_execute_line(line, CLIENT_ALL);
            report_execute_startup_message(startup_lines[n], status, CLIENT_ALL);
        }
    }
}

// ---------------------------------------------------------------------------
// $ commands
// ---------------------------------------------------------------------------
static bool idle_or_alarm() {
    return sys.state == State::Idle || sys.state == State::Alarm;
}

static Error run_sequence(Error (*fn)()) {
    if (sys.state == State::Cycle) {
        protocol_buffer_synchronize();  // Let queued G-code finish first
        if (sys.abort) {
            return Error::SequenceAborted;
        }
    }
    if (sys.state != State::Idle) {
        return Error::IdleError;
    }
    Error e = fn();
    return sys.abort ? Error::SequenceAborted : e;
}

Error system_execute_line(char* line, uint8_t client) {
    // Upper-case and strip spaces, except for the value of $N and $J lines.
    char* cmd = line + 1;
    if (cmd[0] == '\0') {
        report_grbl_help(client);
        return Error::Ok;
    }
    if (cmd[0] == 'J' && cmd[1] == '=') {
        if (!(sys.state == State::Idle || sys.state == State::Jog)) {
            return Error::IdleError;
        }
        return gc_execute_line(line, client);  // Parser starts after "$J="
    }
    char* eq    = strchr(cmd, '=');
    char* value = NULL;
    if (eq) {
        *eq   = '\0';
        value = eq + 1;
    }
    for (char* p = cmd; *p; p++) {
        *p = toupper(*p);
    }
    if (value && !(cmd[0] == 'N')) {
        // Remove whitespace from numeric values
        char* o = value;
        for (char* i = value; *i; i++) {
            if (!isspace((unsigned char)*i)) {
                *o++ = *i;
            }
        }
        *o = '\0';
    }

    if (!value) {
        if (!strcmp(cmd, "$")) {
            if (sys.state == State::Cycle || sys.state == State::Hold || sys.state == State::Jog) {
                return Error::IdleError;
            }
            report_grbl_settings(client);
            return Error::Ok;
        }
        if (!strcmp(cmd, "#")) {
            report_ngc_parameters(client);
            return Error::Ok;
        }
        if (!strcmp(cmd, "G")) {
            report_gcode_modes(client);
            return Error::Ok;
        }
        if (!strcmp(cmd, "I")) {
            report_build_info(client);
            return Error::Ok;
        }
        if (!strcmp(cmd, "N")) {
            for (int n = 0; n < N_STARTUP_LINE; n++) {
                report_startup_line(n, startup_lines[n], client);
            }
            return Error::Ok;
        }
        if (!strcmp(cmd, "C")) {
            if (sys.state == State::CheckMode) {
                mc_reset();
                report_feedback_message(Message::Disabled);
            } else if (sys.state == State::Idle) {
                sys.state = State::CheckMode;
                report_feedback_message(Message::Enabled);
            } else {
                return Error::IdleError;
            }
            return Error::Ok;
        }
        if (!strcmp(cmd, "X")) {
            if (sys.state == State::Alarm) {
                report_feedback_message(Message::AlarmUnlock);
                sys.state = State::Idle;
            }
            return Error::Ok;
        }
        if (!strcmp(cmd, "H") || !strcmp(cmd, "HX")) {
            if (!settings.homing_enable) {
                return Error::SettingDisabled;
            }
            if (!idle_or_alarm()) {
                return Error::IdleError;
            }
            mc_homing_cycle();
            if (!sys.abort) {
                char startup[LINE_BUFFER_SIZE];
                system_execute_startup(startup);
            }
            return Error::Ok;
        }
        if (!strcmp(cmd, "FEED")) {
            return run_sequence(machine_feed_sequence);
        }
        if (!strcmp(cmd, "EJECT")) {
            return run_sequence(machine_eject_sequence);
        }
        // $<number> : print one setting
        uint8_t pos = 0;
        float   num;
        if (read_float(cmd, &pos, &num) && cmd[pos] == '\0') {
            const SettingDesc* d = find_setting((int)num);
            if (!d) {
                return Error::InvalidStatement;
            }
            char out[48];
            format_setting(*d, out, sizeof(out));
            grbl_send(client, out);
            return Error::Ok;
        }
        return Error::InvalidStatement;
    }

    // Commands with a value
    if (!strcmp(cmd, "RST")) {
        if (!idle_or_alarm()) {
            return Error::IdleError;
        }
        if (strcmp(value, "$") && strcmp(value, "#") && strcmp(value, "*")) {
            return Error::InvalidStatement;
        }
        settings_restore(value[0]);
        report_feedback_message(Message::RestoreDefaults);
        mc_reset();
        return Error::Ok;
    }
    if (cmd[0] == 'N' && (cmd[1] == '0' || cmd[1] == '1') && cmd[2] == '\0') {
        if (!idle_or_alarm()) {
            return Error::IdleError;
        }
        int n = cmd[1] - '0';
        if (strlen(value) >= LINE_BUFFER_SIZE - 1) {
            return Error::LineLengthExceeded;
        }
        // Validate the line through the parser (check mode) before storing it.
        if (value[0]) {
            char  tmp[LINE_BUFFER_SIZE];
            State saved = sys.state;
            strlcpy(tmp, value, sizeof(tmp));
            sys.state = State::CheckMode;
            Error e   = gc_execute_line(tmp, client);
            sys.state = saved;
            gc_sync_position();
            if (e != Error::Ok) {
                return e;
            }
        }
        strlcpy(startup_lines[n], value, LINE_BUFFER_SIZE);
        char key[4] = { 'N', cmd[1], 0 };
        protocol_buffer_synchronize();
        prefs.putString(key, startup_lines[n]);
        return Error::Ok;
    }
    uint8_t pos = 0;
    float   num;
    if (read_float(cmd, &pos, &num) && cmd[pos] == '\0') {
        if (!idle_or_alarm()) {
            return Error::IdleError;
        }
        return set_setting((int)num, value);
    }
    return Error::InvalidStatement;
}
