/*
  Machine.cpp - Label cutter I/O and paper handling sequences.

  Copyright (c) 2026 - label cutter port of Grbl_ESP32
  Grbl is free software: you can redistribute it and/or modify it under the
  terms of the GNU General Public License as published by the Free Software
  Foundation, either version 3 of the License, or (at your option) any later
  version. See <http://www.gnu.org/licenses/>.

  Contents
  --------
  * Heads   : two solenoids on LEDC PWM (1 kHz). Mutually exclusive in software
              (the other head is always switched off first, immediately).
  * Fans    : two LEDC PWM outputs (25 kHz), 0-100 %.
  * Sensors : paper-present, rear and front IR sensors on GPIO interrupts.
  * Sequences ($EJECT, $FEED): every "move until sensor X changes" step is
              event driven. The motion is queued as a normal planner move and
              the sensor ISR captures the exact machine position on the edge
              and requests a controlled stop (motion cancel). Follow-up
              distances (DR, DF) are measured from the captured edge position,
              so the deceleration overshoot does not affect accuracy.
  * Feed-motor interlock: Z (feed roller) steps are suppressed in the step ISR
              when there is no paper; a G-code move that needs the feed roller
              is rejected (error 150) or, if the tray runs empty while it is
              executing, the machine enters feed hold.
*/

#include "Grbl.h"
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_rom_gpio.h>
#include <soc/gpio_sig_map.h>
#include <soc/gpio_reg.h>

static_assert(HEAD1_PIN < 32 && HEAD2_PIN < 32, "Head pins must be GPIO0..31 for the ISR-safe kill");

// ---------------------------------------------------------------------------
// Heads
// ---------------------------------------------------------------------------
static const ledc_channel_t HEAD_CH[2]  = { LEDC_CHANNEL_0, LEDC_CHANNEL_1 };
static const int            HEAD_PIN[2] = { HEAD1_PIN, HEAD2_PIN };
static const ledc_channel_t FAN_CH[2]   = { LEDC_CHANNEL_2, LEDC_CHANNEL_3 };
static const int            FAN_PIN[2]  = { FAN1_PIN, FAN2_PIN };
static const uint32_t       HEAD_MAX    = (1u << HEAD_PWM_RES_BITS) - 1;
static const uint32_t       FAN_MAX     = (1u << FAN_PWM_RES_BITS) - 1;

static volatile HeadState head_state = HeadState::Off;
static uint32_t           head_hold_duty;
static esp_timer_handle_t pullin_timer;
static float              fan_percent[2];

static void channel_config(ledc_channel_t ch, int pin, ledc_timer_t timer) {
    ledc_channel_config_t c = {};
    c.gpio_num              = pin;
    c.speed_mode            = LEDC_LOW_SPEED_MODE;
    c.channel               = ch;
    c.intr_type             = LEDC_INTR_DISABLE;
    c.timer_sel             = timer;
    c.duty                  = 0;
    c.hpoint                = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&c));
}

static void head_off(int i) {
    ledc_stop(LEDC_LOW_SPEED_MODE, HEAD_CH[i], 0);  // Immediate, output held low
}

static void head_duty(int i, uint32_t duty) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, HEAD_CH[i], duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, HEAD_CH[i]);
}

static void pullin_done(void* arg) {
    HeadState h = head_state;
    if (h != HeadState::Off && (uintptr_t)arg == (uintptr_t)h) {
        head_duty(h == HeadState::Head1 ? 0 : 1, head_hold_duty);
    }
}

void heads_set(HeadState head, float power) {
    esp_timer_stop(pullin_timer);
    HeadState previous = head_state;
    // Mutual exclusion: both heads are switched off before one is switched on.
    head_off(0);
    head_off(1);
    head_state = HeadState::Off;
    if (head == HeadState::Off) {
        return;
    }
    float frac = settings.head_s_max > 0 ? power / settings.head_s_max : 1.0f;
    frac       = constrain(frac, 0.0f, 1.0f);
    head_hold_duty = lroundf(frac * HEAD_MAX);
    if (head_hold_duty == 0) {
        head_state = head;  // Selected but unpowered (S0)
        return;
    }
    if (previous != HeadState::Off && previous != head && settings.head_switch_ms) {
        delay_msec(settings.head_switch_ms);  // Let the other solenoid release
    }
    int i      = head == HeadState::Head1 ? 0 : 1;
    head_state = head;
    if (settings.head_pullin_ms && head_hold_duty < HEAD_MAX) {
        head_duty(i, HEAD_MAX);  // Full-power pull-in, then drop to the hold duty
        esp_timer_start_once(pullin_timer, (uint64_t)settings.head_pullin_ms * 1000);
    } else {
        head_duty(i, head_hold_duty);
    }
}

void heads_sync(HeadState head, float power) {
    if (sys.state == State::CheckMode) {
        return;
    }
    protocol_buffer_synchronize();  // Head changes happen exactly between moves
    heads_set(head, power);
}

// ISR-safe emergency off: detach both pins from LEDC and drive them low.
void IRAM_ATTR heads_kill() {
    REG_WRITE(GPIO_OUT_W1TC_REG, bit(HEAD1_PIN) | bit(HEAD2_PIN));
    esp_rom_gpio_connect_out_signal(HEAD1_PIN, SIG_GPIO_OUT_IDX, false, false);
    esp_rom_gpio_connect_out_signal(HEAD2_PIN, SIG_GPIO_OUT_IDX, false, false);
    head_state = HeadState::Off;
}

HeadState heads_get_state() {
    return head_state;
}

// ---------------------------------------------------------------------------
// Fans
// ---------------------------------------------------------------------------
bool fans_set(uint8_t index, float percent) {
    if (index > 1) {
        return false;
    }
    percent            = constrain(percent, 0.0f, 100.0f);
    fan_percent[index] = percent;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, FAN_CH[index], lroundf(percent / 100.0f * FAN_MAX));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, FAN_CH[index]);
    return true;
}

float fans_get(uint8_t index) {
    return index < 2 ? fan_percent[index] : 0.0f;
}

// ---------------------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------------------
static const int SENSOR_PIN[3] = { PAPER_SENSOR_PIN, REAR_SENSOR_PIN, FRONT_SENSOR_PIN };

uint8_t IRAM_ATTR machine_sensors() {
    uint32_t in = REG_READ(GPIO_IN_REG);
    uint8_t  s  = 0;
    for (int i = 0; i < 3; i++) {
        bool level  = (in >> SENSOR_PIN[i]) & 1;
        bool active = (settings.sensor_invert & bit(i)) ? level : !level;  // Default active LOW
        if (active) {
            s |= bit(i);
        }
    }
    return s;
}
static_assert(PAPER_SENSOR_PIN < 32 && REAR_SENSOR_PIN < 32 && FRONT_SENSOR_PIN < 32, "Sensor pins must be GPIO0..31");

bool machine_paper_present() {
    return machine_sensors() & SENSOR_PAPER;
}

// ---------------------------------------------------------------------------
// Sequence event watch (armed by a sequence step, evaluated in the sensor ISR)
// ---------------------------------------------------------------------------
struct WatchCond {
    uint8_t mask;
    uint8_t want;
};
static WatchCond       watch_cond[2];
static volatile uint8_t watch_count = 0;   // 0 = disarmed
static volatile int8_t  watch_hit   = -1;  // Index of the condition that fired
static int32_t          watch_pos[N_AXIS]; // Machine position (steps) at the edge
static portMUX_TYPE     watch_mux   = portMUX_INITIALIZER_UNLOCKED;

static volatile const char* seq_name = NULL;
static volatile uint8_t     seq_step = 0;
static volatile bool        paper_out_event = false;

// Must be called with watch_mux held.
static void IRAM_ATTR watch_evaluate(uint8_t s) {
    if (watch_count == 0 || watch_hit >= 0) {
        return;
    }
    for (int i = 0; i < watch_count; i++) {
        if ((s & watch_cond[i].mask) == watch_cond[i].want) {
            for (int a = 0; a < N_AXIS; a++) {
                watch_pos[a] = sys_position[a];
            }
            watch_hit = i;
            if (sys.state == State::Cycle) {
                rt_set(ExecFlag::MotionCancel);  // Controlled stop; position already captured
            }
            break;
        }
    }
}

static void IRAM_ATTR sensor_isr(void* arg) {
    uint8_t s = machine_sensors();
    portENTER_CRITICAL_ISR(&watch_mux);
    watch_evaluate(s);
    portEXIT_CRITICAL_ISR(&watch_mux);

    // Interlock for normal G-code motion: tray ran empty while the feed roller turns.
    if (!(s & SENSOR_PAPER) && seq_name == NULL && (sys.state == State::Cycle || sys.state == State::Jog) &&
        (st_moving_axes() & bit(Z_AXIS))) {
        rt_set(ExecFlag::FeedHold);
        paper_out_event = true;
    }
    protocol_notify_from_isr();
}

static void watch_arm(uint8_t n, WatchCond c0, WatchCond c1) {
    portENTER_CRITICAL(&watch_mux);
    watch_cond[0] = c0;
    watch_cond[1] = c1;
    watch_hit     = -1;
    watch_count   = n;
    watch_evaluate(machine_sensors());  // Condition may already be true: no edge would come
    portEXIT_CRITICAL(&watch_mux);
}

static void watch_disarm() {
    portENTER_CRITICAL(&watch_mux);
    watch_count = 0;
    portEXIT_CRITICAL(&watch_mux);
}

const char* IRAM_ATTR machine_sequence_name() {
    return (const char*)seq_name;
}

uint8_t machine_sequence_step() {
    return seq_step;
}

void machine_poll() {
    if (paper_out_event) {
        paper_out_event = false;
        grbl_msg_sendf(CLIENT_ALL, "Paper out - feed hold");
    }
}

bool machine_feed_resume_allowed() {
    plan_block_t* b = plan_get_current_block();
    return !(b && b->steps[Z_AXIS] && !machine_paper_present());
}

// ---------------------------------------------------------------------------
// Init / reset
// ---------------------------------------------------------------------------
void machine_init() {
    gpio_install_isr_service(0);

    for (int pin : SENSOR_PIN) {
        pinMode(pin, INPUT_PULLUP);
        gpio_set_intr_type((gpio_num_t)pin, GPIO_INTR_ANYEDGE);
        gpio_isr_handler_add((gpio_num_t)pin, sensor_isr, NULL);
        gpio_intr_enable((gpio_num_t)pin);
    }

    ledc_timer_config_t ht = {};
    ht.speed_mode          = LEDC_LOW_SPEED_MODE;
    ht.duty_resolution     = (ledc_timer_bit_t)HEAD_PWM_RES_BITS;
    ht.timer_num           = LEDC_TIMER_0;
    ht.freq_hz             = HEAD_PWM_FREQ;
    ht.clk_cfg             = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&ht));
    ledc_timer_config_t ft = ht;
    ft.duty_resolution     = (ledc_timer_bit_t)FAN_PWM_RES_BITS;
    ft.timer_num           = LEDC_TIMER_1;
    ft.freq_hz             = FAN_PWM_FREQ;
    ESP_ERROR_CHECK(ledc_timer_config(&ft));

    esp_timer_create_args_t targs = {};
    targs.callback                = pullin_done;
    targs.name                    = "headPullIn";
    ESP_ERROR_CHECK(esp_timer_create(&targs, &pullin_timer));

    for (int i = 0; i < 2; i++) {
        channel_config(FAN_CH[i], FAN_PIN[i], LEDC_TIMER_1);
    }
    machine_reset();
}

// Soft reset: heads up and re-attached to LEDC, fans off, sequences cleared.
void machine_reset() {
    if (pullin_timer) {
        esp_timer_stop(pullin_timer);
    }
    heads_kill();
    for (int i = 0; i < 2; i++) {
        channel_config(HEAD_CH[i], HEAD_PIN[i], LEDC_TIMER_0);
        head_off(i);
        fans_set(i, 0);
    }
    head_state = HeadState::Off;
    watch_disarm();
    seq_name = NULL;
    seq_step = 0;
}

// ---------------------------------------------------------------------------
// Sequences
// ---------------------------------------------------------------------------
struct SeqResult {
    Error  error;
    int8_t hit;             // Which watch condition fired (-1 none)
    float  hit_mm[N_AXIS];  // Machine position at the sensor edge
};

static void sync_positions() {
    plan_sync_position();
    gc_sync_position();
}

// Relative Y / Z move that ends early (controlled stop) when one of the
// conditions becomes true. With no conditions it is a plain relative move.
static SeqResult seq_move(float dy, float dz, uint8_t ncond = 0, WatchCond c0 = {}, WatchCond c1 = {}) {
    SeqResult r = { Error::Ok, -1, {} };

    watch_arm(ncond, c0, c1);
    if (ncond && watch_hit >= 0) {  // Already satisfied: nothing to move
        watch_disarm();
        r.hit = watch_hit;
        system_convert_array_steps_to_mpos(r.hit_mm, watch_pos);
        return r;
    }

    float target[N_AXIS];
    memcpy(target, system_get_mpos(), sizeof(target));
    target[Y_AXIS] += dy;
    target[Z_AXIS] += dz;
    float dom = MAX(fabsf(dy), fabsf(dz));
    if (dom > 0) {
        plan_line_data_t pl = {};
        pl.motion.noFeedOverride = 1;
        pl.feed_rate             = settings.seq_speed * sqrtf(dy * dy + dz * dz) / dom;  // Paper speed on the dominant axis
        mc_line(target, &pl);
        rt_set(ExecFlag::CycleStart);
        bool cancel_sent = false;
        while (true) {
            protocol_execute_realtime();
            if (sys.abort) {
                watch_disarm();
                r.error = Error::SequenceAborted;
                return r;
            }
            // Backstop for an edge that arrived while the cycle was not yet running.
            if (watch_hit >= 0 && !cancel_sent && sys.state == State::Cycle) {
                rt_set(ExecFlag::MotionCancel);
                cancel_sent = true;
            }
            if (sys.state == State::Idle && !rt_is(ExecFlag::CycleStart) && plan_get_current_block() == NULL) {
                break;
            }
            if (sys.state == State::Idle && watch_hit >= 0) {
                break;  // Cancelled mid-block; the rest of the block is flushed below
            }
            if (sys.state == State::Alarm) {
                watch_disarm();
                r.error = Error::SequenceAborted;
                return r;
            }
            protocol_wait(1);
        }
    }
    watch_disarm();
    // Flush the rest of the cancelled move and resync (same as Grbl's probe cycle).
    st_reset();
    plan_reset();
    sync_positions();

    if (ncond) {
        r.hit = watch_hit;
        if (r.hit >= 0) {
            system_convert_array_steps_to_mpos(r.hit_mm, watch_pos);
        } else {
            r.error = Error::SensorTimeout;
        }
    }
    return r;
}

// Absolute Y / Z move (machine coordinates) used for DR / DF after an edge.
static Error seq_move_to(float y, float z) {
    float* cur = system_get_mpos();
    return seq_move(y - cur[Y_AXIS], z - cur[Z_AXIS]).error;
}

struct SeqScope {
    SeqScope(const char* name) {
        heads_set(HeadState::Off, 0);
        gc_state.modal.head = HeadState::Off;
        sync_positions();
        seq_step = 0;
        seq_name = name;
    }
    ~SeqScope() {
        seq_name = NULL;
        seq_step = 0;
    }
};

static void zero_yz() {
    if (settings.zero_after_feed) {
        sys_position[Y_AXIS] = 0;
        sys_position[Z_AXIS] = 0;
        sync_positions();
    }
}

// Y forward until both rear and front sensors are clear (+ optional $304).
static Error do_eject() {
    if (!(machine_sensors() & (SENSOR_REAR | SENSOR_FRONT))) {
        return Error::Ok;
    }
    SeqResult r = seq_move(settings.seq_max_search, 0, 1, { SENSOR_REAR | SENSOR_FRONT, 0 });
    if (r.error != Error::Ok) {
        return r.error;
    }
    if (settings.eject_extra > 0) {
        return seq_move(settings.eject_extra, 0).error;
    }
    return Error::Ok;
}

Error machine_eject_sequence() {
    SeqScope scope("EJECT");
    seq_step = 1;
    if (!(machine_sensors() & (SENSOR_REAR | SENSOR_FRONT))) {
        grbl_msg_sendf(CLIENT_ALL, "No sheet to eject");
        return Error::Ok;
    }
    Error e = do_eject();
    if (e == Error::Ok) {
        zero_yz();
        grbl_msg_sendf(CLIENT_ALL, "Eject complete");
    }
    return e;
}

Error machine_feed_sequence() {
    SeqScope  scope("FEED");
    SeqResult r;
    const float S = settings.seq_max_search;

    // 1. Eject a sheet that is still in the machine.
    seq_step = 1;
    Error e  = do_eject();
    if (e != Error::Ok) {
        return e;
    }
    // 2. There must be paper in the tray.
    seq_step = 2;
    if (!machine_paper_present()) {
        return Error::NoPaper;
    }
    // 3. Feed + Y forward until the rear sensor is covered (abort if the tray runs empty).
    seq_step = 3;
    r        = seq_move(S, S, 2, { SENSOR_REAR, SENSOR_REAR }, { SENSOR_PAPER, 0 });
    if (r.error != Error::Ok) {
        return r.error;
    }
    if (r.hit == 1) {
        return Error::PaperOutDuringFeed;
    }
    // 4. Feed + Y travel DR measured from the rear-sensor edge.
    seq_step = 4;
    e        = seq_move_to(r.hit_mm[Y_AXIS] + settings.feed_dr, r.hit_mm[Z_AXIS] + settings.feed_dr);
    if (e != Error::Ok) {
        return e;
    }
    // 5. Feed stops; Y forward until the rear sensor clears (sheet tail passed).
    seq_step = 5;
    r        = seq_move(S, 0, 1, { SENSOR_REAR, 0 });
    if (r.error != Error::Ok) {
        return r.error;
    }
    // 6. Y reverse until the front sensor clears (sheet lead edge at the front sensor).
    seq_step = 6;
    if (!(machine_sensors() & SENSOR_FRONT)) {
        return Error::FrontSensorNotCovered;
    }
    r = seq_move(-S, 0, 1, { SENSOR_FRONT, 0 });
    if (r.error != Error::Ok) {
        return r.error;
    }
    // 7. Y travel DF measured from the front-sensor edge.
    seq_step = 7;
    e        = seq_move_to(r.hit_mm[Y_AXIS] + settings.feed_df, system_get_mpos()[Z_AXIS]);
    if (e != Error::Ok) {
        return e;
    }
    // 8. Done. Y = 0 is now the registered sheet position.
    seq_step = 8;
    zero_yz();
    grbl_msg_sendf(CLIENT_ALL, "Feed complete");
    return Error::Ok;
}
