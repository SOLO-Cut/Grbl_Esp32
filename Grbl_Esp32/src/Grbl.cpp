/*
  Grbl.cpp - Initialization, soft reset and shared globals.

  Part of Grbl
  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC
  2018 - Bart Dring (ESP32 port)

  Grbl is free software: you can redistribute it and/or modify it under the
  terms of the GNU General Public License as published by the Free Software
  Foundation, either version 3 of the License, or (at your option) any later
  version. See <http://www.gnu.org/licenses/>.

  This build contains no WiFi, Bluetooth, web, telnet, SD or OTA code. The
  radio libraries are never linked, so the radios stay off; communication is
  serial only (UART0 console + UART1 master link).
*/

#include "Grbl.h"

system_t                  sys;
std::atomic<uint32_t>     sys_rt_exec_state(0);
volatile ExecAlarm        sys_rt_exec_alarm = ExecAlarm::None;
volatile uint8_t          sys_rt_f_override = FEED_OVERRIDE_DEFAULT;
volatile uint8_t          sys_rt_r_override = RAPID_OVERRIDE_DEFAULT;
volatile bool             cycle_stop        = false;
volatile int32_t          sys_position[N_AXIS];
volatile void*            sys_pl_data_inflight = NULL;

void grbl_init() {
    serial_init();
    settings_init();
    machine_init();  // Installs the GPIO ISR service, heads, fans, sensors
    stepper_init();
    limits_init();
    for (int i = 0; i < N_AXIS; i++) {
        sys_position[i] = 0;
    }
    grbl_msg_sendf(CLIENT_ALL, "Grbl_ESP32 Ver %s Date %s", GRBL_VERSION, GRBL_VERSION_BUILD);
    grbl_msg_sendf(CLIENT_ALL, "Compiled with ESP32 SDK:%s", ESP.getSdkVersion());
    grbl_msg_sendf(CLIENT_ALL, "Using machine:%s", MACHINE_NAME);
#ifdef HOMING_INIT_LOCK
    sys.state = settings.homing_enable ? State::Alarm : State::Idle;
#else
    sys.state = State::Idle;
#endif
}

static void reset_variables() {
    State prior_state = sys.state;
    memset(&sys, 0, sizeof(system_t));
    sys.state      = prior_state;
    sys.f_override = FEED_OVERRIDE_DEFAULT;
    sys.r_override = RAPID_OVERRIDE_DEFAULT;

    sys_rt_exec_state.store(0);
    sys_rt_exec_alarm = ExecAlarm::None;
    cycle_stop        = false;
    sys_rt_f_override = FEED_OVERRIDE_DEFAULT;
    sys_rt_r_override = RAPID_OVERRIDE_DEFAULT;

    serial_reset_read_buffer(CLIENT_ALL);
    gc_init();
    machine_reset();
    plan_reset();
    st_reset();
    plan_sync_position();
    gc_sync_position();
    report_init_message(CLIENT_ALL);
    sys_pl_data_inflight = NULL;
}

void run_once() {
    reset_variables();
    protocol_main_loop();  // Returns on a soft reset (ctrl-x, $RST, alarms)
}

// Reads a floating point value from a string (Grbl's fast, locale free parser).
bool read_float(const char* line, uint8_t* char_counter, float* float_ptr) {
    const char*   ptr = line + *char_counter;
    unsigned char c   = *ptr++;
    bool          isnegative = false;
    if (c == '-') {
        isnegative = true;
        c          = *ptr++;
    } else if (c == '+') {
        c = *ptr++;
    }
    uint32_t intval    = 0;
    int8_t   exp       = 0;
    uint8_t  ndigit    = 0;
    bool     isdecimal = false;
    while (1) {
        c -= '0';
        if (c <= 9) {
            ndigit++;
            if (ndigit <= 8) {
                if (isdecimal) {
                    exp--;
                }
                intval = intval * 10 + c;
            } else if (!isdecimal) {
                exp++;
            }
        } else if (c == (('.' - '0') & 0xff) && !isdecimal) {
            isdecimal = true;
        } else {
            break;
        }
        c = *ptr++;
    }
    if (!ndigit) {
        return false;
    }
    float fval = (float)intval;
    if (fval != 0) {
        while (exp <= -2) {
            fval *= 0.01f;
            exp += 2;
        }
        if (exp < 0) {
            fval *= 0.1f;
        } else if (exp > 0) {
            do {
                fval *= 10.0f;
            } while (--exp > 0);
        }
    }
    *float_ptr    = isnegative ? -fval : fval;
    *char_counter = ptr - line - 1;
    return true;
}
