/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl) with additions from Sungeun K. Jeon (https://github.com/chamnit/grbl)
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
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
#include "StepTicker.h"
#include "modules/tools/flybyvision/FlyByVision.h"

#include <math.h>
#include <algorithm>

#define junction_deviation_checksum    CHECKSUM("junction_deviation")
#define z_junction_deviation_checksum  CHECKSUM("z_junction_deviation")
#define minimum_planner_speed_checksum CHECKSUM("minimum_planner_speed")
#define s_curve_enable_checksum        CHECKSUM("s_curve_enable")
#define max_jerk_checksum              CHECKSUM("max_jerk")
#define path_blending_enable_checksum  CHECKSUM("path_blending_enable")
#define path_tolerance_checksum        CHECKSUM("path_tolerance")

static inline void finalize_flyby_trigger(Block *block)
{
    if(block != nullptr && block->flyby_trigger.enabled()) {
        FlyByVision::finalize_trigger_tick(block->flyby_trigger,
                                           block->millimeters,
                                           block->entry_speed,
                                           block->exit_speed,
                                           block->maximum_rate,
                                           block->steps_event_count,
                                           block->accelerate_until,
                                           block->decelerate_after,
                                           block->total_move_ticks,
                                           THEKERNEL->step_ticker->get_frequency());
    }
}

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

    // Disabled by default so existing Smoothieware configurations retain their exact behaviour.
    this->s_curve_enable = THEKERNEL->config->value(s_curve_enable_checksum)->by_default(false)->as_bool();
    this->max_jerk = THEKERNEL->config->value(max_jerk_checksum)->by_default(0.0F)->as_number();
    this->path_blending_enable = THEKERNEL->config->value(path_blending_enable_checksum)->by_default(false)->as_bool();
    this->path_tolerance = THEKERNEL->config->value(path_tolerance_checksum)->by_default(0.0F)->as_number();
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

    if(!has_steps) {
        block->clear();
        return true;
    }

    block->s_value = roundf(s_value*(1<<11));
    block->is_g123 = g123;

    float junction_deviation = this->junction_deviation;
    if(z_only && !isnan(this->z_junction_deviation)) junction_deviation = this->z_junction_deviation;

    // Path tolerance is an explicit geometric allowance for fast transit moves.
    // It reuses Smoothie's proven centripetal junction model, but only to increase the
    // permitted corner deviation. It does not silently alter the commanded endpoints.
    if(this->path_blending_enable && block->primary_axis && this->path_tolerance > junction_deviation) {
        junction_deviation = this->path_tolerance;
    }

    block->acceleration = acceleration;
    block->s_curve_jerk = (this->s_curve_enable && this->max_jerk > 0.0F) ? this->max_jerk : 0.0F;
    block->s_curve_active = false;

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
    block->nominal_length_flag = (block->nominal_speed <= v_allowable);
    block->recalculate_flag = true;

    if(unit_vec != nullptr) memcpy(previous_unit_vec, unit_vec, sizeof(previous_unit_vec));
    else memset(previous_unit_vec, 0, sizeof(previous_unit_vec));

    this->recalculate();
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
            finalize_flyby_trigger(previous);
        }
    }

    current->calculate_trapezoid(current->entry_speed, minimum_planner_speed);
    finalize_flyby_trigger(current);
}

float Planner::max_allowable_speed(float acceleration, float target_velocity, float distance)
{
    return(sqrtf(target_velocity * target_velocity - 2.0F * acceleration * distance));
}
