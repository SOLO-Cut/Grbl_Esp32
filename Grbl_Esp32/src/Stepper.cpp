/*
  Stepper.cpp - Executes the motion plan with the stepper motors.

  Part of Grbl
  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  2018 - Bart Dring (ESP32 port)

  Grbl is free software: you can redistribute it and/or modify it under the
  terms of the GNU General Public License as published by the Free Software
  Foundation, either version 3 of the License, or (at your option) any later
  version. See <http://www.gnu.org/licenses/>.

  ESP32-S3 port notes
  -------------------
  * The step ISR is driven by an ESP-IDF 5 "gptimer" alarm (20 MHz resolution,
    auto reload). Each alarm executes one Bresenham/AMASS tick; the alarm
    period is reprogrammed from inside the ISR whenever a new segment starts.
  * STEP/DIR/ENABLE are written with single GPIO set/clear register writes.
  * Whenever a segment is consumed the ISR notifies the main task, which
    refills the segment buffer (event driven instead of busy polling).
  * The feed-motor interlock lives here: Z steps are suppressed in hardware
    whenever the paper sensor reports "no paper".
*/

#include "Grbl.h"
#include <driver/gptimer.h>
#include <soc/gpio_reg.h>
#include <esp_cpu.h>

static_assert(X_STEP_PIN < 32 && Y_STEP_PIN < 32 && Z_STEP_PIN < 32, "Step pins must be GPIO0..31");
static_assert(X_DIRECTION_PIN < 32 && Y_DIRECTION_PIN < 32 && Z_DIRECTION_PIN < 32, "Dir pins must be GPIO0..31");
static_assert(X_ENABLE_PIN < 49 && Y_ENABLE_PIN < 49 && Z_ENABLE_PIN < 49, "Enable pins must be GPIO0..48");
static_assert(X_ENABLE_PIN != Y_ENABLE_PIN && X_ENABLE_PIN != Z_ENABLE_PIN && Y_ENABLE_PIN != Z_ENABLE_PIN,
              "Each driver needs its own enable pin");
static_assert(PAPER_SENSOR_PIN < 32, "Paper sensor must be GPIO0..31");

// Step timer
static const uint32_t fStepperTimer       = 20000000;  // Hz
static const uint32_t amassThreshold      = fStepperTimer / 8000;
static const int      maxAmassLevel       = 3;
static const float    DT_SEGMENT          = 1.0f / (ACCELERATION_TICKS_PER_SECOND * 60.0f);  // min/segment
static const float    REQ_MM_INCREMENT_SCALAR = 1.25f;
static const int      RAMP_ACCEL          = 0;
static const int      RAMP_CRUISE         = 1;
static const int      RAMP_DECEL          = 2;
static const int      RAMP_DECEL_OVERRIDE = 3;

static const uint32_t step_pin_mask[N_AXIS] = { bit(X_STEP_PIN), bit(Y_STEP_PIN), bit(Z_STEP_PIN) };
static const uint32_t dir_pin_mask[N_AXIS]  = { bit(X_DIRECTION_PIN), bit(Y_DIRECTION_PIN), bit(Z_DIRECTION_PIN) };
static const uint8_t  enable_pin[N_AXIS]    = { X_ENABLE_PIN, Y_ENABLE_PIN, Z_ENABLE_PIN };

struct PrepFlag {
    uint8_t recalculate : 1;
    uint8_t holdPartialBlock : 1;
    uint8_t decelOverride : 1;
};

typedef struct {
    uint32_t steps[N_AXIS];
    uint32_t step_event_count;
    uint8_t  direction_bits;
} st_block_t;
static st_block_t st_block_buffer[SEGMENT_BUFFER_SIZE - 1];

typedef struct {
    uint16_t n_step;
    uint16_t isrPeriod;
    uint8_t  st_block_index;
    uint8_t  amass_level;
} segment_t;
static segment_t segment_buffer[SEGMENT_BUFFER_SIZE];

typedef struct {
    uint32_t    counter[N_AXIS];
    uint8_t     step_outbits;
    uint8_t     dir_outbits;
    uint8_t     last_dir_outbits;
    uint32_t    steps[N_AXIS];
    uint16_t    step_count;
    uint8_t     exec_block_index;
    st_block_t* exec_block;
    segment_t*  exec_segment;
    AxisMask    moving_axes;
} stepper_t;
static stepper_t st;

static volatile uint8_t segment_buffer_tail;
static uint8_t          segment_buffer_head;
static uint8_t          segment_next_head;

static plan_block_t* pl_block;
static st_block_t*   st_prep_block;

typedef struct {
    uint8_t  st_block_index;
    PrepFlag recalculate_flag;
    float    dt_remainder;
    float    steps_remaining;
    float    step_per_mm;
    float    req_mm_increment;
    uint8_t  ramp_type;
    float    mm_complete;
    float    current_speed;
    float    maximum_speed;
    float    exit_speed;
    float    accelerate_until;
    float    decelerate_after;
} st_prep_t;
static st_prep_t prep;

static gptimer_handle_t step_timer      = NULL;
static volatile bool    timer_running   = false;
static uint16_t         current_period  = 0;
static int64_t          idle_disable_at = 0;  // esp_timer time to disable drivers (0 = none)

// ---------------------------------------------------------------------------
// Low-level I/O
// ---------------------------------------------------------------------------
static inline void IRAM_ATTR write_mask(uint32_t set, uint32_t clr) {
    if (set) {
        REG_WRITE(GPIO_OUT_W1TS_REG, set);
    }
    if (clr) {
        REG_WRITE(GPIO_OUT_W1TC_REG, clr);
    }
}

static inline void IRAM_ATTR set_step_pins(uint8_t active) {
    uint32_t set = 0, clr = 0;
    for (int axis = 0; axis < N_AXIS; axis++) {
        bool level = ((active >> axis) & 1) ^ ((settings.step_invert >> axis) & 1);
        if (level) {
            set |= step_pin_mask[axis];
        } else {
            clr |= step_pin_mask[axis];
        }
    }
    write_mask(set, clr);
}

static inline void IRAM_ATTR set_dir_pins(uint8_t dirbits) {
    uint32_t set = 0, clr = 0;
    for (int axis = 0; axis < N_AXIS; axis++) {
        bool level = ((dirbits >> axis) & 1) ^ ((settings.dir_invert >> axis) & 1);
        if (level) {
            set |= dir_pin_mask[axis];
        } else {
            clr |= dir_pin_mask[axis];
        }
    }
    write_mask(set, clr);
}

static inline bool IRAM_ATTR paper_present_fast() {
    bool level = (REG_READ(GPIO_IN_REG) >> PAPER_SENSOR_PIN) & 1;
    bool active_low = !(settings.sensor_invert & SENSOR_PAPER);
    return active_low ? !level : level;
}

static inline void IRAM_ATTR busy_wait_us(uint32_t us) {
    uint32_t start  = esp_cpu_get_cycle_count();
    uint32_t cycles = us * (uint32_t)(F_CPU / 1000000);
    while ((esp_cpu_get_cycle_count() - start) < cycles) {
    }
}

// Drives one GPIO from ISR context; handles both banks (GPIO0..31 and 32..48).
static inline void IRAM_ATTR write_pin(uint8_t pin, bool level) {
    if (pin < 32) {
        REG_WRITE(level ? GPIO_OUT_W1TS_REG : GPIO_OUT_W1TC_REG, bit(pin));
    } else {
        REG_WRITE(level ? GPIO_OUT1_W1TS_REG : GPIO_OUT1_W1TC_REG, bit(pin - 32));
    }
}

void IRAM_ATTR st_disable_motors(bool disable) {
    for (int axis = 0; axis < N_AXIS; axis++) {
        // Default ($4 bit = 0): HIGH = disabled (active-low enable)
        bool level = disable ^ (((settings.enable_invert >> axis) & 1) != 0);
        write_pin(enable_pin[axis], level);
    }
}

static void IRAM_ATTR timer_set_period(uint16_t ticks) {
    if (ticks == current_period) {
        return;
    }
    current_period              = ticks;
    gptimer_alarm_config_t alarm = {};
    alarm.alarm_count            = ticks;
    alarm.reload_count           = 0;
    alarm.flags.auto_reload_on_alarm = true;
    gptimer_set_alarm_action(step_timer, &alarm);
}

static void IRAM_ATTR timer_stop() {
    if (timer_running) {
        timer_running = false;
        gptimer_stop(step_timer);
    }
}

static void timer_start() {
    if (!timer_running) {
        timer_running = true;
        gptimer_set_raw_count(step_timer, 0);
        gptimer_start(step_timer);
    }
}

AxisMask st_moving_axes() {
    return timer_running ? st.moving_axes : 0;
}

// ---------------------------------------------------------------------------
// The stepper driver interrupt (one Bresenham / AMASS tick)
// ---------------------------------------------------------------------------
static bool IRAM_ATTR stepper_pulse_func() {
    bool notify = false;

    // Direction first, then (optionally) wait for the driver setup time.
    if (st.dir_outbits != st.last_dir_outbits) {
        set_dir_pins(st.dir_outbits);
        st.last_dir_outbits = st.dir_outbits;
        if (settings.dir_delay_us) {
            busy_wait_us(settings.dir_delay_us);
        }
    }

    // Step pulse for the bits computed during the previous tick.
    uint32_t pulse_start = esp_cpu_get_cycle_count();
    set_step_pins(st.step_outbits);
    bool pulsing = st.step_outbits != 0;

    if (st.exec_segment == NULL) {
        if (segment_buffer_head != segment_buffer_tail) {
            st.exec_segment = &segment_buffer[segment_buffer_tail];
            timer_set_period(st.exec_segment->isrPeriod);
            st.step_count = st.exec_segment->n_step;
            if (st.exec_block_index != st.exec_segment->st_block_index) {
                st.exec_block_index = st.exec_segment->st_block_index;
                st.exec_block       = &st_block_buffer[st.exec_block_index];
                st.moving_axes      = 0;
                for (int axis = 0; axis < N_AXIS; axis++) {
                    st.counter[axis] = (st.exec_block->step_event_count >> 1);
                    if (st.exec_block->steps[axis]) {
                        st.moving_axes |= bit(axis);
                    }
                }
                // Feed-motor interlock for normal G-code motion: a block that needs
                // the feed roller while the tray is empty puts the machine in hold.
                if ((st.moving_axes & bit(Z_AXIS)) && !paper_present_fast() && machine_sequence_name() == NULL) {
                    rt_set(ExecFlag::FeedHold);
                    notify = true;
                }
            }
            st.dir_outbits = st.exec_block->direction_bits;
            for (int axis = 0; axis < N_AXIS; axis++) {
                st.steps[axis] = st.exec_block->steps[axis] >> st.exec_segment->amass_level;
            }
        } else {
            // Segment buffer empty: shut down.
            if (pulsing) {
                while ((esp_cpu_get_cycle_count() - pulse_start) < settings.pulse_us * (uint32_t)(F_CPU / 1000000)) {
                }
            }
            st_go_idle();
            cycle_stop = true;
            return true;
        }
    }

    st.step_outbits = 0;
    for (int axis = 0; axis < N_AXIS; axis++) {
        st.counter[axis] += st.steps[axis];
        if (st.counter[axis] > st.exec_block->step_event_count) {
            st.counter[axis] -= st.exec_block->step_event_count;
            st.step_outbits |= bit(axis);
        }
    }
    if (sys.state == State::Homing) {
        st.step_outbits &= sys.homing_axis_lock;
    }
    // The feed roller never turns without paper (applies to every motion type).
    if ((st.step_outbits & bit(Z_AXIS)) && !paper_present_fast()) {
        st.step_outbits &= ~bit(Z_AXIS);
    }
    for (int axis = 0; axis < N_AXIS; axis++) {
        if (st.step_outbits & bit(axis)) {
            if (st.exec_block->direction_bits & bit(axis)) {
                sys_position[axis] = sys_position[axis] - 1;
            } else {
                sys_position[axis] = sys_position[axis] + 1;
            }
        }
    }

    if (--st.step_count == 0) {
        st.exec_segment = NULL;
        uint8_t tail    = segment_buffer_tail + 1;
        segment_buffer_tail = (tail == SEGMENT_BUFFER_SIZE) ? 0 : tail;
        notify          = true;  // Main loop: refill the segment buffer
    }

    // Finish the step pulse that was started at the top of the ISR.
    if (pulsing) {
        uint32_t cycles = settings.pulse_us * (uint32_t)(F_CPU / 1000000);
        while ((esp_cpu_get_cycle_count() - pulse_start) < cycles) {
        }
        set_step_pins(0);
    }
    return notify;
}

static bool IRAM_ATTR onStepTimer(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx) {
    if (stepper_pulse_func()) {
        protocol_notify_from_isr();
    }
    return false;  // protocol_notify_from_isr() yields if required
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------
void stepper_init() {
    st_disable_motors(true);  // Latch "disabled" before the enable pins become outputs
    const int pins[] = { X_STEP_PIN,   Y_STEP_PIN,   Z_STEP_PIN,  X_DIRECTION_PIN, Y_DIRECTION_PIN,
                         Z_DIRECTION_PIN, X_ENABLE_PIN, Y_ENABLE_PIN, Z_ENABLE_PIN };
    for (int pin : pins) {
        pinMode(pin, OUTPUT);
    }
    set_step_pins(0);
    set_dir_pins(0);
    st_disable_motors(true);

    gptimer_config_t cfg = {};
    cfg.clk_src          = GPTIMER_CLK_SRC_DEFAULT;
    cfg.direction        = GPTIMER_COUNT_UP;
    cfg.resolution_hz    = fStepperTimer;
    ESP_ERROR_CHECK(gptimer_new_timer(&cfg, &step_timer));
    gptimer_event_callbacks_t cbs = {};
    cbs.on_alarm                  = onStepTimer;
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(step_timer, &cbs, NULL));
    current_period = 0;
    timer_set_period(0xffff);
    ESP_ERROR_CHECK(gptimer_enable(step_timer));
}

void st_wake_up() {
    st_disable_motors(false);
    idle_disable_at = 0;
    timer_start();
}

void IRAM_ATTR st_go_idle() {
    timer_stop();
    // Hold the motors for $1 ms after motion to let the axes settle; 255 keeps them on.
    if ((settings.idle_lock_ms != 0xff || sys_rt_exec_alarm != ExecAlarm::None) && sys.state != State::Homing) {
        if (sys_rt_exec_alarm != ExecAlarm::None) {
            st_disable_motors(true);
        } else {
            idle_disable_at = esp_timer_get_time() + (int64_t)settings.idle_lock_ms * 1000 + 1;
        }
    } else {
        st_disable_motors(false);
    }
    set_step_pins(0);
    st.step_outbits = 0;
}

// Called by the main loop: performs the delayed driver disable.
void st_check_idle_disable() {
    int64_t at = idle_disable_at;
    if (at && !timer_running && esp_timer_get_time() > at) {
        idle_disable_at = 0;
        st_disable_motors(true);
    }
}

void st_reset() {
    st_go_idle();
    memset(&prep, 0, sizeof(st_prep_t));
    uint8_t last_dir = st.last_dir_outbits;
    memset(&st, 0, sizeof(stepper_t));
    st.last_dir_outbits = last_dir;
    st.exec_segment     = NULL;
    pl_block            = NULL;
    segment_buffer_tail = 0;
    segment_buffer_head = 0;
    segment_next_head   = 1;
}

void st_update_plan_block_parameters() {
    if (pl_block != NULL) {
        prep.recalculate_flag.recalculate = 1;
        pl_block->entry_speed_sqr         = prep.current_speed * prep.current_speed;
        pl_block                          = NULL;
    }
}

static uint8_t st_next_block_index(uint8_t block_index) {
    block_index++;
    return block_index == (SEGMENT_BUFFER_SIZE - 1) ? 0 : block_index;
}

// Fills the segment buffer from the planner. Called from the main loop whenever
// the step ISR reports that a segment was consumed (and on every realtime pass).
void st_prep_buffer() {
    if (sys.step_control.endMotion) {
        return;
    }
    while (segment_buffer_tail != segment_next_head) {
        if (pl_block == NULL) {
            pl_block = sys.step_control.executeSysMotion ? plan_get_system_motion_block() : plan_get_current_block();
            if (pl_block == NULL) {
                return;
            }
            if (prep.recalculate_flag.recalculate) {
                prep.recalculate_flag = {};
            } else {
                prep.st_block_index           = st_next_block_index(prep.st_block_index);
                st_prep_block                 = &st_block_buffer[prep.st_block_index];
                st_prep_block->direction_bits = pl_block->direction_bits;
                for (int idx = 0; idx < N_AXIS; idx++) {
                    st_prep_block->steps[idx] = pl_block->steps[idx] << maxAmassLevel;
                }
                st_prep_block->step_event_count = pl_block->step_event_count << maxAmassLevel;
                prep.steps_remaining            = (float)pl_block->step_event_count;
                prep.step_per_mm                = prep.steps_remaining / pl_block->millimeters;
                prep.req_mm_increment           = REQ_MM_INCREMENT_SCALAR / prep.step_per_mm;
                prep.dt_remainder               = 0.0f;
                if ((sys.step_control.executeHold) || prep.recalculate_flag.decelOverride) {
                    prep.current_speed                  = prep.exit_speed;
                    pl_block->entry_speed_sqr           = prep.exit_speed * prep.exit_speed;
                    prep.recalculate_flag.decelOverride = 0;
                } else {
                    prep.current_speed = sqrtf(pl_block->entry_speed_sqr);
                }
            }
            // Velocity profile for the (new or updated) block.
            prep.mm_complete  = 0.0f;
            float inv_2_accel = 0.5f / pl_block->acceleration;
            if (sys.step_control.executeHold) {
                prep.ramp_type   = RAMP_DECEL;
                float decel_dist = pl_block->millimeters - inv_2_accel * pl_block->entry_speed_sqr;
                if (decel_dist < 0.0f) {
                    prep.exit_speed = sqrtf(pl_block->entry_speed_sqr - 2 * pl_block->acceleration * pl_block->millimeters);
                } else {
                    prep.mm_complete = decel_dist;
                    prep.exit_speed  = 0.0f;
                }
            } else {
                prep.ramp_type        = RAMP_ACCEL;
                prep.accelerate_until = pl_block->millimeters;
                float exit_speed_sqr;
                if (sys.step_control.executeSysMotion) {
                    prep.exit_speed = exit_speed_sqr = 0.0f;
                } else {
                    exit_speed_sqr  = plan_get_exec_block_exit_speed_sqr();
                    prep.exit_speed = sqrtf(exit_speed_sqr);
                }
                float nominal_speed      = plan_compute_profile_nominal_speed(pl_block);
                float nominal_speed_sqr  = nominal_speed * nominal_speed;
                float intersect_distance = 0.5f * (pl_block->millimeters + inv_2_accel * (pl_block->entry_speed_sqr - exit_speed_sqr));
                if (pl_block->entry_speed_sqr > nominal_speed_sqr) {
                    prep.accelerate_until = pl_block->millimeters - inv_2_accel * (pl_block->entry_speed_sqr - nominal_speed_sqr);
                    if (prep.accelerate_until <= 0.0f) {
                        prep.ramp_type = RAMP_DECEL;
                        prep.exit_speed = sqrtf(pl_block->entry_speed_sqr - 2 * pl_block->acceleration * pl_block->millimeters);
                        prep.recalculate_flag.decelOverride = 1;
                    } else {
                        prep.decelerate_after = inv_2_accel * (nominal_speed_sqr - exit_speed_sqr);
                        prep.maximum_speed    = nominal_speed;
                        prep.ramp_type        = RAMP_DECEL_OVERRIDE;
                    }
                } else if (intersect_distance > 0.0f) {
                    if (intersect_distance < pl_block->millimeters) {
                        prep.decelerate_after = inv_2_accel * (nominal_speed_sqr - exit_speed_sqr);
                        if (prep.decelerate_after < intersect_distance) {
                            prep.maximum_speed = nominal_speed;
                            if (pl_block->entry_speed_sqr == nominal_speed_sqr) {
                                prep.ramp_type = RAMP_CRUISE;
                            } else {
                                prep.accelerate_until -= inv_2_accel * (nominal_speed_sqr - pl_block->entry_speed_sqr);
                            }
                        } else {
                            prep.accelerate_until = intersect_distance;
                            prep.decelerate_after = intersect_distance;
                            prep.maximum_speed    = sqrtf(2.0f * pl_block->acceleration * intersect_distance + exit_speed_sqr);
                        }
                    } else {
                        prep.ramp_type = RAMP_DECEL;
                    }
                } else {
                    prep.accelerate_until = 0.0f;
                    prep.maximum_speed    = prep.exit_speed;
                }
            }
        }

        segment_t* prep_segment      = &segment_buffer[segment_buffer_head];
        prep_segment->st_block_index = prep.st_block_index;

        float dt_max       = DT_SEGMENT;
        float dt           = 0.0f;
        float time_var     = dt_max;
        float mm_var;
        float speed_var;
        float mm_remaining = pl_block->millimeters;
        float minimum_mm   = mm_remaining - prep.req_mm_increment;
        if (minimum_mm < 0.0f) {
            minimum_mm = 0.0f;
        }
        do {
            switch (prep.ramp_type) {
                case RAMP_DECEL_OVERRIDE:
                    speed_var = pl_block->acceleration * time_var;
                    mm_var    = time_var * (prep.current_speed - 0.5f * speed_var);
                    mm_remaining -= mm_var;
                    if ((mm_remaining < prep.accelerate_until) || (mm_var <= 0)) {
                        mm_remaining       = prep.accelerate_until;
                        time_var           = 2.0f * (pl_block->millimeters - mm_remaining) / (prep.current_speed + prep.maximum_speed);
                        prep.ramp_type     = RAMP_CRUISE;
                        prep.current_speed = prep.maximum_speed;
                    } else {
                        prep.current_speed -= speed_var;
                    }
                    break;
                case RAMP_ACCEL:
                    speed_var = pl_block->acceleration * time_var;
                    mm_remaining -= time_var * (prep.current_speed + 0.5f * speed_var);
                    if (mm_remaining < prep.accelerate_until) {
                        mm_remaining       = prep.accelerate_until;
                        time_var           = 2.0f * (pl_block->millimeters - mm_remaining) / (prep.current_speed + prep.maximum_speed);
                        prep.ramp_type     = (mm_remaining == prep.decelerate_after) ? RAMP_DECEL : RAMP_CRUISE;
                        prep.current_speed = prep.maximum_speed;
                    } else {
                        prep.current_speed += speed_var;
                    }
                    break;
                case RAMP_CRUISE:
                    mm_var = mm_remaining - prep.maximum_speed * time_var;
                    if (mm_var < prep.decelerate_after) {
                        time_var       = (mm_remaining - prep.decelerate_after) / prep.maximum_speed;
                        mm_remaining   = prep.decelerate_after;
                        prep.ramp_type = RAMP_DECEL;
                    } else {
                        mm_remaining = mm_var;
                    }
                    break;
                default:  // RAMP_DECEL
                    speed_var = pl_block->acceleration * time_var;
                    if (prep.current_speed > speed_var) {
                        mm_var = mm_remaining - time_var * (prep.current_speed - 0.5f * speed_var);
                        if (mm_var > prep.mm_complete) {
                            mm_remaining = mm_var;
                            prep.current_speed -= speed_var;
                            break;
                        }
                    }
                    time_var           = 2.0f * (mm_remaining - prep.mm_complete) / (prep.current_speed + prep.exit_speed);
                    mm_remaining       = prep.mm_complete;
                    prep.current_speed = prep.exit_speed;
            }
            dt += time_var;
            if (dt < dt_max) {
                time_var = dt_max - dt;
            } else {
                if (mm_remaining > minimum_mm) {
                    dt_max += DT_SEGMENT;
                    time_var = dt_max - dt;
                } else {
                    break;
                }
            }
        } while (mm_remaining > prep.mm_complete);

        float step_dist_remaining    = prep.step_per_mm * mm_remaining;
        float n_steps_remaining      = ceilf(step_dist_remaining);
        float last_n_steps_remaining = ceilf(prep.steps_remaining);
        prep_segment->n_step         = last_n_steps_remaining - n_steps_remaining;

        if (prep_segment->n_step == 0) {
            if (sys.step_control.executeHold) {
                sys.step_control.endMotion             = true;
                prep.recalculate_flag.holdPartialBlock = 1;
                return;
            }
        }

        dt += prep.dt_remainder;
        float    inv_rate   = dt / (last_n_steps_remaining - step_dist_remaining);
        uint32_t timerTicks = ceilf((fStepperTimer * 60.0f) * inv_rate);
        int      level;
        for (level = 0; level < maxAmassLevel; level++) {
            if (timerTicks < amassThreshold) {
                break;
            }
            timerTicks >>= 1;
        }
        prep_segment->amass_level = level;
        prep_segment->n_step <<= level;
        prep_segment->isrPeriod = timerTicks > 0xffff ? 0xffff : timerTicks;

        segment_buffer_head = segment_next_head;
        if (++segment_next_head == SEGMENT_BUFFER_SIZE) {
            segment_next_head = 0;
        }
        pl_block->millimeters = mm_remaining;
        prep.steps_remaining  = n_steps_remaining;
        prep.dt_remainder     = (n_steps_remaining - step_dist_remaining) * inv_rate;
        if (mm_remaining == prep.mm_complete) {
            if (mm_remaining > 0.0f) {
                sys.step_control.endMotion             = true;
                prep.recalculate_flag.holdPartialBlock = 1;
                return;
            } else {
                if (sys.step_control.executeSysMotion) {
                    sys.step_control.endMotion = true;
                    return;
                }
                pl_block = NULL;
                plan_discard_current_block();
            }
        }
    }
}

float st_get_realtime_rate() {
    switch (sys.state) {
        case State::Cycle:
        case State::Homing:
        case State::Hold:
        case State::Jog:
            return prep.current_speed;
        default:
            return 0.0f;
    }
}
