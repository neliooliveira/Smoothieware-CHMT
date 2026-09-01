/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <bitset>
#include "ActuatorCoordinates.h"
#include "modules/tools/flybyvision/FlyByTrigger.h"

class Block {
    public:
        Block();

        static void init(uint8_t);

        void calculate_trapezoid( float entry_speed, float exit_speed );

        float reverse_pass(float exit_speed);
        float forward_pass(float next_entry_speed);
        float max_exit_speed();
        void debug() const;
        void ready() { is_ready= true; }
        void clear();
        float get_trapezoid_rate(int i) const;

    private:
        float max_allowable_speed( float acceleration, float target_velocity, float distance);
        float max_allowable_speed_jerk(float target_velocity, float distance) const;
        void prepare(float acceleration_in_steps, float deceleration_in_steps);
        void calculate_s_curve(float entry_speed, float exit_speed);
        void prepare_s_curve(float accel_jerk_steps_s3, float decel_jerk_steps_s3);

        static double fp_scale;

    public:
        std::array<uint32_t, k_max_actuators> steps;
        uint32_t steps_event_count;
        float nominal_rate;
        float nominal_speed;
        float millimeters;
        float entry_speed;
        float exit_speed;
        float acceleration;
        float initial_rate;
        float maximum_rate;

        float max_entry_speed;

        uint32_t accelerate_until;
        uint32_t decelerate_after;
        uint32_t total_move_ticks;
        std::bitset<k_max_actuators> direction_bits;

        // Jerk-limited 7-phase S-curve. s_curve_jerk is in mm/s^3.
        // A value <= 0 selects the legacy trapezoidal acceleration profile.
        float s_curve_jerk;
        bool s_curve_active;
        uint32_t s_curve_phase_end[7];

        FlyByTrigger flyby_trigger;

        using tickinfo_t= struct {
            int64_t steps_per_tick;
            int64_t counter;
            int64_t acceleration_change;
            int64_t deceleration_change;
            int64_t plateau_rate;
            int64_t jerk_change;
            int64_t accel_jerk_change;
            int64_t decel_jerk_change;
            uint32_t steps_to_move;
            uint32_t step_count;
            uint32_t next_accel_event;
        };

        tickinfo_t *tick_info;

        static uint8_t n_actuators;

        struct {
            bool recalculate_flag:1;
            bool nominal_length_flag:1;
            bool is_ready:1;
            bool primary_axis:1;
            bool is_g123:1;
            volatile bool is_ticking:1;
            volatile bool locked:1;
            uint16_t s_value:12;
        };
};
