/*
  Planner.cpp - Buffers movement commands and manages the acceleration profile plan

  Part of Grbl
  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011 Jens Geisler
  2018 - Bart Dring (ESP32 port)

  Grbl is free software: you can redistribute it and/or modify it under the
  terms of the GNU General Public License as published by the Free Software
  Foundation, either version 3 of the License, or (at your option) any later
  version. See <http://www.gnu.org/licenses/>.

  The algorithm is unchanged from Grbl 1.1. See the Grbl source for the long
  description of the forward / reverse planning passes.
*/

#include "Grbl.h"

static plan_block_t block_buffer[BLOCK_BUFFER_SIZE];  // Ring buffer of motion instructions
static uint8_t      block_buffer_tail;                // Block being executed
static uint8_t      block_buffer_head;                // Next block to be pushed
static uint8_t      next_buffer_head;
static uint8_t      block_buffer_planned;             // First block that is not optimally planned

typedef struct {
    int32_t position[N_AXIS];  // Planner position in absolute steps
    float   previous_unit_vec[N_AXIS];
    float   previous_nominal_speed;
} planner_t;
static planner_t pl;

uint8_t plan_next_block_index(uint8_t block_index) {
    block_index++;
    return block_index == BLOCK_BUFFER_SIZE ? 0 : block_index;
}

static uint8_t plan_prev_block_index(uint8_t block_index) {
    if (block_index == 0) {
        block_index = BLOCK_BUFFER_SIZE;
    }
    return block_index - 1;
}

static float convert_delta_vector_to_unit_vector(float* vector) {
    float magnitude = 0.0f;
    for (int idx = 0; idx < N_AXIS; idx++) {
        magnitude += vector[idx] * vector[idx];
    }
    magnitude           = sqrtf(magnitude);
    float inv_magnitude = 1.0f / magnitude;
    for (int idx = 0; idx < N_AXIS; idx++) {
        vector[idx] *= inv_magnitude;
    }
    return magnitude;
}

static float limit_acceleration_by_axis_maximum(const float* unit_vec) {
    float limit_value = SOME_LARGE_VALUE;
    for (int idx = 0; idx < N_AXIS; idx++) {
        if (unit_vec[idx] != 0) {
            limit_value = MIN(limit_value, fabsf(settings.acceleration[idx] / unit_vec[idx]));
        }
    }
    return limit_value * SEC_PER_MIN_SQ;  // mm/s^2 -> mm/min^2
}

static float limit_rate_by_axis_maximum(const float* unit_vec) {
    float limit_value = SOME_LARGE_VALUE;
    for (int idx = 0; idx < N_AXIS; idx++) {
        if (unit_vec[idx] != 0) {
            limit_value = MIN(limit_value, fabsf(settings.max_rate[idx] / unit_vec[idx]));
        }
    }
    return limit_value;
}

static void planner_recalculate() {
    uint8_t block_index = plan_prev_block_index(block_buffer_head);
    if (block_index == block_buffer_planned) {
        return;  // Only one plannable block
    }
    // Reverse pass
    float         entry_speed_sqr;
    plan_block_t* next;
    plan_block_t* current    = &block_buffer[block_index];
    current->entry_speed_sqr = MIN(current->max_entry_speed_sqr, 2 * current->acceleration * current->millimeters);
    block_index              = plan_prev_block_index(block_index);
    if (block_index == block_buffer_planned) {
        if (block_index == block_buffer_tail) {
            st_update_plan_block_parameters();
        }
    } else {
        while (block_index != block_buffer_planned) {
            next        = current;
            current     = &block_buffer[block_index];
            block_index = plan_prev_block_index(block_index);
            if (block_index == block_buffer_tail) {
                st_update_plan_block_parameters();
            }
            if (current->entry_speed_sqr != current->max_entry_speed_sqr) {
                entry_speed_sqr = next->entry_speed_sqr + 2 * current->acceleration * current->millimeters;
                current->entry_speed_sqr = entry_speed_sqr < current->max_entry_speed_sqr ? entry_speed_sqr : current->max_entry_speed_sqr;
            }
        }
    }
    // Forward pass
    next        = &block_buffer[block_buffer_planned];
    block_index = plan_next_block_index(block_buffer_planned);
    while (block_index != block_buffer_head) {
        current = next;
        next    = &block_buffer[block_index];
        if (current->entry_speed_sqr < next->entry_speed_sqr) {
            entry_speed_sqr = current->entry_speed_sqr + 2 * current->acceleration * current->millimeters;
            if (entry_speed_sqr < next->entry_speed_sqr) {
                next->entry_speed_sqr = entry_speed_sqr;
                block_buffer_planned  = block_index;
            }
        }
        if (next->entry_speed_sqr == next->max_entry_speed_sqr) {
            block_buffer_planned = block_index;
        }
        block_index = plan_next_block_index(block_index);
    }
}

void plan_reset() {
    memset(&pl, 0, sizeof(planner_t));
    plan_reset_buffer();
}

void plan_reset_buffer() {
    block_buffer_tail    = 0;
    block_buffer_head    = 0;
    next_buffer_head     = 1;
    block_buffer_planned = 0;
}

void plan_discard_current_block() {
    if (block_buffer_head != block_buffer_tail) {
        uint8_t block_index = plan_next_block_index(block_buffer_tail);
        if (block_buffer_tail == block_buffer_planned) {
            block_buffer_planned = block_index;
        }
        block_buffer_tail = block_index;
    }
}

plan_block_t* plan_get_system_motion_block() {
    return &block_buffer[block_buffer_head];
}

plan_block_t* plan_get_current_block() {
    if (block_buffer_head == block_buffer_tail) {
        return NULL;
    }
    return &block_buffer[block_buffer_tail];
}

float plan_get_exec_block_exit_speed_sqr() {
    uint8_t block_index = plan_next_block_index(block_buffer_tail);
    if (block_index == block_buffer_head) {
        return 0.0f;
    }
    return block_buffer[block_index].entry_speed_sqr;
}

bool plan_check_full_buffer() {
    return block_buffer_tail == next_buffer_head;
}

float plan_compute_profile_nominal_speed(plan_block_t* block) {
    float nominal_speed = block->programmed_rate;
    if (block->motion.rapidMotion) {
        nominal_speed *= (0.01f * sys.r_override);
    } else {
        if (!(block->motion.noFeedOverride)) {
            nominal_speed *= (0.01f * sys.f_override);
        }
        if (nominal_speed > block->rapid_rate) {
            nominal_speed = block->rapid_rate;
        }
    }
    return nominal_speed > MINIMUM_FEED_RATE ? nominal_speed : MINIMUM_FEED_RATE;
}

static void plan_compute_profile_parameters(plan_block_t* block, float nominal_speed, float prev_nominal_speed) {
    if (nominal_speed > prev_nominal_speed) {
        block->max_entry_speed_sqr = prev_nominal_speed * prev_nominal_speed;
    } else {
        block->max_entry_speed_sqr = nominal_speed * nominal_speed;
    }
    if (block->max_entry_speed_sqr > block->max_junction_speed_sqr) {
        block->max_entry_speed_sqr = block->max_junction_speed_sqr;
    }
}

void plan_update_velocity_profile_parameters() {
    uint8_t block_index        = block_buffer_tail;
    float   prev_nominal_speed = SOME_LARGE_VALUE;
    while (block_index != block_buffer_head) {
        plan_block_t* block         = &block_buffer[block_index];
        float         nominal_speed = plan_compute_profile_nominal_speed(block);
        plan_compute_profile_parameters(block, nominal_speed, prev_nominal_speed);
        prev_nominal_speed = nominal_speed;
        block_index        = plan_next_block_index(block_index);
    }
    pl.previous_nominal_speed = prev_nominal_speed;
}

// Add a new linear movement to the buffer. target[] is the absolute target in mm.
// Returns false for a zero-length block.
bool plan_buffer_line(float* target, plan_line_data_t* pl_data) {
    plan_block_t* block = &block_buffer[block_buffer_head];
    memset(block, 0, sizeof(plan_block_t));
    block->motion      = pl_data->motion;
    block->line_number = pl_data->line_number;

    int32_t target_steps[N_AXIS], position_steps[N_AXIS];
    float   unit_vec[N_AXIS], delta_mm;
    if (block->motion.systemMotion) {
        for (int i = 0; i < N_AXIS; i++) {
            position_steps[i] = sys_position[i];
        }
    } else {
        memcpy(position_steps, pl.position, sizeof(pl.position));
    }
    for (int idx = 0; idx < N_AXIS; idx++) {
        target_steps[idx]       = lroundf(target[idx] * settings.steps_per_mm[idx]);
        block->steps[idx]       = labs(target_steps[idx] - position_steps[idx]);
        block->step_event_count = MAX(block->step_event_count, block->steps[idx]);
        delta_mm                = (target_steps[idx] - position_steps[idx]) / settings.steps_per_mm[idx];
        unit_vec[idx]           = delta_mm;
        if (delta_mm < 0.0f) {
            block->direction_bits |= bit(idx);  // Set bit = negative direction
        }
    }
    if (block->step_event_count == 0) {
        return false;
    }

    block->millimeters  = convert_delta_vector_to_unit_vector(unit_vec);
    block->acceleration = limit_acceleration_by_axis_maximum(unit_vec);
    block->rapid_rate   = limit_rate_by_axis_maximum(unit_vec);
    if (block->motion.rapidMotion) {
        block->programmed_rate = block->rapid_rate;
    } else {
        block->programmed_rate = pl_data->feed_rate;
        if (block->motion.inverseTime) {
            block->programmed_rate *= block->millimeters;
        }
    }

    if ((block_buffer_head == block_buffer_tail) || (block->motion.systemMotion)) {
        block->entry_speed_sqr        = 0.0f;
        block->max_junction_speed_sqr = 0.0f;
    } else {
        // Junction speed by centripetal acceleration approximation (junction deviation).
        float junction_unit_vec[N_AXIS];
        float junction_cos_theta = 0.0f;
        for (int idx = 0; idx < N_AXIS; idx++) {
            junction_cos_theta -= pl.previous_unit_vec[idx] * unit_vec[idx];
            junction_unit_vec[idx] = unit_vec[idx] - pl.previous_unit_vec[idx];
        }
        if (junction_cos_theta > 0.999999f) {
            block->max_junction_speed_sqr = MINIMUM_JUNCTION_SPEED * MINIMUM_JUNCTION_SPEED;
        } else if (junction_cos_theta < -0.999999f) {
            block->max_junction_speed_sqr = SOME_LARGE_VALUE;
        } else {
            convert_delta_vector_to_unit_vector(junction_unit_vec);
            float junction_acceleration = limit_acceleration_by_axis_maximum(junction_unit_vec);
            float sin_theta_d2          = sqrtf(0.5f * (1.0f - junction_cos_theta));
            block->max_junction_speed_sqr =
                MAX(MINIMUM_JUNCTION_SPEED * MINIMUM_JUNCTION_SPEED,
                    (junction_acceleration * settings.junction_dev * sin_theta_d2) / (1.0f - sin_theta_d2));
        }
    }

    if (!(block->motion.systemMotion)) {
        float nominal_speed = plan_compute_profile_nominal_speed(block);
        plan_compute_profile_parameters(block, nominal_speed, pl.previous_nominal_speed);
        pl.previous_nominal_speed = nominal_speed;
        memcpy(pl.previous_unit_vec, unit_vec, sizeof(unit_vec));
        memcpy(pl.position, target_steps, sizeof(target_steps));
        block_buffer_head = next_buffer_head;
        next_buffer_head  = plan_next_block_index(block_buffer_head);
        planner_recalculate();
    }
    return true;
}

void plan_sync_position() {
    for (int idx = 0; idx < N_AXIS; idx++) {
        pl.position[idx] = sys_position[idx];
    }
}

uint8_t plan_get_block_buffer_available() {
    if (block_buffer_head >= block_buffer_tail) {
        return (BLOCK_BUFFER_SIZE - 1) - (block_buffer_head - block_buffer_tail);
    }
    return block_buffer_tail - block_buffer_head - 1;
}

void plan_cycle_reinitialize() {
    st_update_plan_block_parameters();
    block_buffer_planned = block_buffer_tail;
    planner_recalculate();
}
