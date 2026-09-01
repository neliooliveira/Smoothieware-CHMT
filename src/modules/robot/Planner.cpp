/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl) with additions from Sungeun K. Jeon (https://github.com/chamnit/grbl)
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
*/

using namespace std;

#include "mri.h"
#include "nuts_bolts.h"
#include "RingBuffer.h"
#include "Gcode.h"
#include "Module.h"
#include "Kernel.h"
#include "Block.h"
#include "Planner.h"
#include "Conveyor.h"
#include "StepperMotor.h"
#include "Config.h"
#include "checksumm.h"
#include "Robot.h"
#include "ConfigValue.h"
#include "modules/tools/flybyvision/FlyByVision.h"

#include <math.h>
#include <algorithm>

#define junction_deviation_checksum    CHECKSUM("junction_deviation")
#define z_junction_deviation_checksum  CHECKSUM("z_junction_deviation")
#define minimum_planner_speed_checksum CHECKSUM("minimum_planner_speed")

Planner::Planner()
{
    memset(this->previous_unit_vec, 0, sizeof this->previous_unit_vec);
    config_load();
}

void Planner::config_load()
{
    this->junction_deviation = THEKERNEL->config->value(junction_deviation_checksum)->by_default(0.05F)->as_number();
    this->z_junction_deviation = THEKERNEL->config->value(z_junction_deviation_checksum)->by_default(NAN)->as_number();
    this->minimum_planner_speed = THEKERNEL->config->value(minimum_planner_speed_checksum)->by_default(0.0f)->as_number();
}

bool Planner::append_block( ActuatorCoordinates &actuator_pos, uint8_t n_motors, float rate_mm_s, float distance, float *unit_vec, float acceleration, float s_value, bool g123)
{
    Block* block = THECONVEYOR->queue.head_ref();
    bool has_steps = false;
    bool z_only = true;
    block->primary_axis = false;
    for (size_t i = 0; i < n_motors; i++) {
        int32_t steps = THEROBOT->actuators[i]->steps_to_target(actuator_pos[i]);
        if(steps != 0) {
            THEROBOT->actuators[i]->update_last_milestones(actuator_pos[i], steps);
            has_steps = true;
            if (i < N_PRIMARY_AXIS) block->primary_axis = true;
        }
        block->direction_bits[i] = (steps < 0) ? 1 : 0;
        block->steps[i] = labs(steps);
        if (i != GAMMA_STEPPER && block->steps[i] != 0) z_only = false;
    }
    if (block->steps[GAMMA_STEPPER] == 0) z_only = false;

    if(!has_steps) { block->clear(); return true; }

    block->s_value = roundf(s_value*(1<<11));
    block->is_g123 = g123;
    float junction_deviation = this->junction_deviation;
    if(z_only && !isnan(this->z_junction_deviation)) junction_deviation = this->z_junction_deviation;
    block->acceleration = acceleration;

    auto mi = std::max_element(block->steps.begin(), block->steps.end());
    block->steps_event_count = *mi;
    block->millimeters = distance;
    if(distance > 0.0F) {
        block->nominal_speed = rate_mm_s;
        block->nominal_rate = block->steps_event_count * rate_mm_s / distance;
    } else {
        block->nominal_speed = 0.0F;
        block->nominal_rate = 0;
    }

    // Consume an M950 arm exactly once, on the next real motion block.
    if(FlyByVision::getInstance() != nullptr) {
        FlyByVision::getInstance()->consume_pending(block->flyby_trigger, distance);
    }

    float vmax_junction = minimum_planner_speed;
    if (unit_vec != nullptr && !THECONVEYOR->is_queue_empty()) {
        Block *prev_block = THECONVEYOR->queue.item_ref(THECONVEYOR->queue.prev(THECONVEYOR->queue.head_i));
        if (junction_deviation > 0.0F && prev_block->primary_axis == block->primary_axis && prev_block->nominal_speed > 0.0F) {
            float cos_theta = - this->previous_unit_vec[X_AXIS] * unit_vec[X_AXIS]
                              - this->previous_unit_vec[Y_AXIS] * unit_vec[Y_AXIS]
                              - this->previous_unit_vec[Z_AXIS] * unit_vec[Z_AXIS];
            for (int i = 3; i < n_motors; ++i) cos_theta -= this->previous_unit_vec[i] * unit_vec[i];
            if (cos_theta <= 0.9999F) {
                vmax_junction = std::min(prev_block->nominal_speed, block->nominal_speed);
                if (cos_theta >= -0.9999F) {
                    float sin_theta_d2 = sqrtf(0.5F * (1.0F - cos_theta));
                    float max_acceleration = std::min(prev_block->acceleration, block->acceleration);
                    vmax_junction = std::min(vmax_junction, sqrtf(max_acceleration * junction_deviation * sin_theta_d2 / (1.0F - sin_theta_d2)));
                }
            }
        }
    }
    block->max_entry_speed = vmax_junction;
    float v_allowable = max_allowable_speed(-acceleration, minimum_planner_speed, block->millimeters);
    block->entry_speed = std::min(vmax_junction, v_allowable);
    block->nominal_length_flag = block->nominal_speed <= v_allowable;
    block->recalculate_flag = true;

    if(unit_vec != nullptr) memcpy(previous_unit_vec, unit_vec, sizeof(previous_unit_vec));
    else memset(previous_unit_vec, 0, sizeof(previous_unit_vec));

    this->recalculate();

    // M950 stores a normalized spatial fraction in trigger_tick until the
    // final trapezoid is known. Convert it to the nearest execution tick.
    if(block->flyby_trigger.enabled() && block->total_move_ticks > 0) {
        const double fraction = (double)block->flyby_trigger.trigger_tick / 4294967295.0;
        uint32_t tick = (uint32_t)(fraction * (double)(block->total_move_ticks - 1));
        block->flyby_trigger.trigger_tick = tick;
    }

    block->ready();
    THECONVEYOR->queue_head_block();
    return true;
}

void Planner::recalculate()
{
    Conveyor::Queue_t &queue = THECONVEYOR->queue;
    unsigned int block_index;
    Block* previous;
    Block* current;
    float entry_speed = minimum_planner_speed;
    block_index = queue.head_i;
    current = queue.item_ref(block_index);

    if (!queue.is_empty()) {
        while ((block_index != queue.tail_i) && current->recalculate_flag) {
            entry_speed = current->reverse_pass(entry_speed);
            block_index = queue.prev(block_index);
            current = queue.item_ref(block_index);
        }
        float exit_speed = current->max_exit_speed();
        while (block_index != queue.head_i) {
            previous = current;
            block_index = queue.next(block_index);
            current = queue.item_ref(block_index);
            exit_speed = current->forward_pass(exit_speed);
            previous->calculate_trapezoid(previous->entry_speed, current->entry_speed);
        }
    }
    current->calculate_trapezoid(current->entry_speed, minimum_planner_speed);
}

float Planner::max_allowable_speed(float acceleration, float target_velocity, float distance)
{
    return(sqrtf(target_velocity * target_velocity - 2.0F * acceleration * distance));
}
