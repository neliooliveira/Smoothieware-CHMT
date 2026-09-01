/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef PLANNER_H
#define PLANNER_H

#include "ActuatorCoordinates.h"
class Block;

class Planner
{
public:
    Planner();
    float max_allowable_speed( float acceleration, float target_velocity, float distance);

    // Pick & Place path-control API. G61/G64 are handled by the PnP motion
    // extension module so OpenPnP can switch modes without rebuilding config.
    void set_exact_path(bool exact) { path_blending_enable = !exact; }
    void set_path_tolerance(float tolerance) {
        path_tolerance = tolerance < 0.0F ? 0.0F : tolerance;
        path_blending_enable = path_tolerance > 0.0F;
    }
    bool is_path_blending_enabled() const { return path_blending_enable; }
    float get_path_tolerance() const { return path_tolerance; }
    bool is_s_curve_enabled() const { return s_curve_enable && max_jerk > 0.0F; }
    float get_max_jerk() const { return max_jerk; }

    friend class Robot;

private:
    bool append_block(ActuatorCoordinates &target, uint8_t n_motors, float rate_mm_s, float distance, float unit_vec[], float accleration, float s_value, bool g123);
    void recalculate();
    void config_load();
    float previous_unit_vec[MAX_ROBOT_ACTUATORS];
    float junction_deviation;
    float z_junction_deviation;
    float minimum_planner_speed;

    // Optional Pick & Place motion extensions.
    bool s_curve_enable;
    float max_jerk;              // physical jerk limit, mm/s^3
    bool path_blending_enable;
    float path_tolerance;        // maximum cornering deviation used by junction model, mm
};

#endif
