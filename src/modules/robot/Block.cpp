/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include <cmath>
#include <string>
#include <algorithm>
#include "Block.h"
#include "Planner.h"
#include "Conveyor.h"
#include "Gcode.h"
#include "libs/StreamOutputPool.h"
#include "StepTicker.h"
#include "platform_memory.h"

#include "mri.h"
#include <inttypes.h>

using std::string;

#define STEP_TICKER_FREQUENCY THEKERNEL->step_ticker->get_frequency()

uint8_t Block::n_actuators= 0;
double Block::fp_scale= 0;

namespace {
struct SCurveRamp {
    double tj;
    double ta;
    double duration;
    double distance;
};

static SCurveRamp make_scurve_ramp(double v0, double v1, double amax, double jerk)
{
    SCurveRamp r{0.0, 0.0, 0.0, 0.0};
    const double dv = std::max(0.0, v1 - v0);
    if(dv <= 0.0 || amax <= 0.0 || jerk <= 0.0) return r;

    const double dv_at_amax = (amax * amax) / jerk;
    if(dv <= dv_at_amax) {
        r.tj = sqrt(dv / jerk);
        r.ta = 0.0;
    } else {
        r.tj = amax / jerk;
        r.ta = dv / amax - r.tj;
    }

    r.duration = 2.0 * r.tj + r.ta;
    // A zero-acceleration-ended S ramp is time symmetric, so its total
    // distance is average velocity times duration.
    r.distance = 0.5 * (v0 + v1) * r.duration;
    return r;
}

static uint32_t seconds_to_ticks(double seconds, double frequency)
{
    if(seconds <= 0.0) return 0;
    uint32_t ticks = (uint32_t)llround(seconds * frequency);
    return ticks == 0 ? 1 : ticks;
}
}

Block::Block()
{
    tick_info= nullptr;
    clear();
}

void Block::init(uint8_t n)
{
    n_actuators= n;
    fp_scale= (double)STEPTICKER_FPSCALE / pow((double)STEP_TICKER_FREQUENCY, 2.0);
}

void Block::clear()
{
    is_ready            = false;
    this->steps.fill(0);
    steps_event_count   = 0;
    nominal_rate        = 0.0F;
    nominal_speed       = 0.0F;
    millimeters         = 0.0F;
    entry_speed         = 0.0F;
    exit_speed          = 0.0F;
    acceleration        = 100.0F;
    initial_rate        = 0.0F;
    maximum_rate        = 0.0F;
    accelerate_until    = 0;
    decelerate_after    = 0;
    direction_bits      = 0;
    recalculate_flag    = false;
    nominal_length_flag = false;
    max_entry_speed     = 0.0F;
    is_ticking          = false;
    is_g123             = false;
    locked              = false;
    s_value             = 0.0F;
    total_move_ticks    = 0;
    s_curve_jerk        = 0.0F;
    s_curve_active      = false;
    for(int i = 0; i < 7; ++i) s_curve_phase_end[i] = 0;
    flyby_trigger.clear();

    if(tick_info == nullptr) {
        tick_info= new tickinfo_t[n_actuators];
        if(tick_info == nullptr) __debugbreak();
    }

    for(int i = 0; i < n_actuators; ++i) {
        tick_info[i].steps_per_tick= 0;
        tick_info[i].counter= 0;
        tick_info[i].acceleration_change= 0;
        tick_info[i].deceleration_change= 0;
        tick_info[i].plateau_rate= 0;
        tick_info[i].jerk_change= 0;
        tick_info[i].accel_jerk_change= 0;
        tick_info[i].decel_jerk_change= 0;
        tick_info[i].steps_to_move= 0;
        tick_info[i].step_count= 0;
        tick_info[i].next_accel_event= 0;
    }
}

void Block::debug() const
{
    THEKERNEL->streams->printf("%p: steps-X:%lu Y:%lu Z:%lu ", this, this->steps[0], this->steps[1], this->steps[2]);
    for (size_t i = E_AXIS; i < n_actuators; ++i) {
        THEKERNEL->streams->printf("%c:%lu ", 'A' + i-E_AXIS, this->steps[i]);
    }
    THEKERNEL->streams->printf("(max:%lu) nominal:r%1.4f/s%1.4f mm:%1.4f acc:%1.2f accu:%lu decu:%lu ticks:%lu rates:%1.4f/%1.4f entry/max:%1.4f/%1.4f exit:%1.4f scurve:%d jerk:%1.2f primary:%d ready:%d locked:%d ticking:%d recalc:%d nomlen:%d time:%f\r\n",
                               this->steps_event_count,
                               this->nominal_rate,
                               this->nominal_speed,
                               this->millimeters,
                               this->acceleration,
                               this->accelerate_until,
                               this->decelerate_after,
                               this->total_move_ticks,
                               this->initial_rate,
                               this->maximum_rate,
                               this->entry_speed,
                               this->max_entry_speed,
                               this->exit_speed,
                               this->s_curve_active ? 1 : 0,
                               this->s_curve_jerk,
                               this->primary_axis,
                               this->is_ready,
                               this->locked,
                               this->is_ticking,
                               recalculate_flag ? 1 : 0,
                               nominal_length_flag ? 1 : 0,
                               total_move_ticks/STEP_TICKER_FREQUENCY);
}

void Block::calculate_trapezoid( float entryspeed, float exitspeed )
{
    if (is_ticking) return;

    if(s_curve_jerk > 0.0F && millimeters > 0.0F && nominal_speed > 0.0F) {
        calculate_s_curve(entryspeed, exitspeed);
        return;
    }

    s_curve_active = false;

    float initial_rate = this->nominal_rate * (entryspeed / this->nominal_speed);
    float final_rate = this->nominal_rate * (exitspeed / this->nominal_speed);
    float acceleration_per_second = (this->acceleration * this->steps_event_count) / this->millimeters;

    float maximum_possible_rate = sqrtf((this->steps_event_count * acceleration_per_second) + ((powf(initial_rate, 2) + powf(final_rate, 2)) / 2.0F));
    this->maximum_rate = std::min(maximum_possible_rate, this->nominal_rate);

    float time_to_accelerate = (this->maximum_rate - initial_rate) / acceleration_per_second;
    float time_to_decelerate = (final_rate - this->maximum_rate) / -acceleration_per_second;
    float plateau_time = 0;

    if(maximum_possible_rate > this->nominal_rate) {
        float acceleration_distance = ((initial_rate + this->maximum_rate) / 2.0F) * time_to_accelerate;
        float deceleration_distance = ((this->maximum_rate + final_rate) / 2.0F) * time_to_decelerate;
        float plateau_distance = this->steps_event_count - acceleration_distance - deceleration_distance;
        plateau_time = plateau_distance / this->maximum_rate;
    }

    float total_move_time = time_to_accelerate + time_to_decelerate + plateau_time;
    uint32_t acceleration_ticks = floorf(time_to_accelerate * STEP_TICKER_FREQUENCY);
    uint32_t deceleration_ticks = floorf(time_to_decelerate * STEP_TICKER_FREQUENCY);
    uint32_t total_ticks = floorf(total_move_time * STEP_TICKER_FREQUENCY);

    float acceleration_time = acceleration_ticks / STEP_TICKER_FREQUENCY;
    float deceleration_time = deceleration_ticks / STEP_TICKER_FREQUENCY;
    float acceleration_in_steps = (acceleration_time > 0.0F) ? (this->maximum_rate - initial_rate) / acceleration_time : 0;
    float deceleration_in_steps = (deceleration_time > 0.0F) ? (this->maximum_rate - final_rate) / deceleration_time : 0;

    this->locked= true;
    this->accelerate_until = acceleration_ticks;
    this->decelerate_after = total_ticks - deceleration_ticks;
    this->total_move_ticks = total_ticks;
    this->initial_rate = initial_rate;
    this->exit_speed = exitspeed;
    this->prepare(acceleration_in_steps, deceleration_in_steps);
    this->locked= false;
}

void Block::calculate_s_curve(float entryspeed, float exitspeed)
{
    const double frequency = STEP_TICKER_FREQUENCY;
    const double distance_steps = (double)steps_event_count;
    const double v0 = (double)nominal_rate * ((double)entryspeed / (double)nominal_speed);
    const double vf = (double)nominal_rate * ((double)exitspeed / (double)nominal_speed);
    const double amax = ((double)acceleration * distance_steps) / (double)millimeters;
    const double jerk = ((double)s_curve_jerk * distance_steps) / (double)millimeters;

    if(distance_steps <= 0.0 || amax <= 0.0 || jerk <= 0.0 || frequency <= 0.0) {
        float saved = s_curve_jerk;
        s_curve_jerk = 0.0F;
        calculate_trapezoid(entryspeed, exitspeed);
        s_curve_jerk = saved;
        return;
    }

    double peak = std::max(v0, vf);
    double high = std::max(peak, (double)nominal_rate);

    SCurveRamp low_acc = make_scurve_ramp(v0, peak, amax, jerk);
    SCurveRamp low_dec = make_scurve_ramp(vf, peak, amax, jerk);
    if(low_acc.distance + low_dec.distance > distance_steps + 1.0e-6) {
        // The look-ahead is still acceleration based. If its entry/exit speeds
        // cannot be connected under the requested jerk limit, preserve the
        // proven trapezoidal profile rather than generating an impossible move.
        float saved = s_curve_jerk;
        s_curve_jerk = 0.0F;
        calculate_trapezoid(entryspeed, exitspeed);
        s_curve_jerk = saved;
        return;
    }

    SCurveRamp acc = make_scurve_ramp(v0, high, amax, jerk);
    SCurveRamp dec = make_scurve_ramp(vf, high, amax, jerk);
    if(acc.distance + dec.distance <= distance_steps) {
        peak = high;
    } else {
        double lo = peak;
        double hi = high;
        for(int i = 0; i < 48; ++i) {
            const double mid = 0.5 * (lo + hi);
            SCurveRamp a = make_scurve_ramp(v0, mid, amax, jerk);
            SCurveRamp d = make_scurve_ramp(vf, mid, amax, jerk);
            if(a.distance + d.distance <= distance_steps) lo = mid;
            else hi = mid;
        }
        peak = lo;
        acc = make_scurve_ramp(v0, peak, amax, jerk);
        dec = make_scurve_ramp(vf, peak, amax, jerk);
    }

    const double plateau_distance = std::max(0.0, distance_steps - acc.distance - dec.distance);
    const double plateau_time = peak > 0.0 ? plateau_distance / peak : 0.0;

    const uint32_t ja = seconds_to_ticks(acc.tj, frequency);
    const uint32_t ca = seconds_to_ticks(acc.ta, frequency);
    const uint32_t pl = seconds_to_ticks(plateau_time, frequency);
    const uint32_t jd = seconds_to_ticks(dec.tj, frequency);
    const uint32_t cd = seconds_to_ticks(dec.ta, frequency);

    this->locked = true;
    s_curve_phase_end[0] = ja;
    s_curve_phase_end[1] = s_curve_phase_end[0] + ca;
    s_curve_phase_end[2] = s_curve_phase_end[1] + ja;
    s_curve_phase_end[3] = s_curve_phase_end[2] + pl;
    s_curve_phase_end[4] = s_curve_phase_end[3] + jd;
    s_curve_phase_end[5] = s_curve_phase_end[4] + cd;
    s_curve_phase_end[6] = s_curve_phase_end[5] + jd;

    total_move_ticks = s_curve_phase_end[6];
    accelerate_until = s_curve_phase_end[2];
    decelerate_after = s_curve_phase_end[3];
    initial_rate = (float)v0;
    maximum_rate = (float)peak;
    exit_speed = exitspeed;
    s_curve_active = true;
    prepare_s_curve((float)jerk, (float)jerk);
    this->locked = false;
}

float Block::max_allowable_speed(float acceleration, float target_velocity, float distance)
{
    return sqrtf(target_velocity * target_velocity - 2.0F * acceleration * distance);
}

float Block::reverse_pass(float exit_speed)
{
    if (this->entry_speed != this->max_entry_speed) {
        if ((!this->nominal_length_flag) && (this->max_entry_speed > exit_speed)) {
            float max_entry_speed = max_allowable_speed(-this->acceleration, exit_speed, this->millimeters);
            this->entry_speed = min(max_entry_speed, this->max_entry_speed);
            return this->entry_speed;
        } else {
            this->entry_speed = this->max_entry_speed;
        }
    }
    return this->entry_speed;
}

float Block::forward_pass(float prev_max_exit_speed)
{
    if (prev_max_exit_speed > nominal_speed) prev_max_exit_speed = nominal_speed;
    if (prev_max_exit_speed > max_entry_speed) prev_max_exit_speed = max_entry_speed;

    if (prev_max_exit_speed <= entry_speed) {
        entry_speed = prev_max_exit_speed;
        recalculate_flag = false;
    }
    return max_exit_speed();
}

float Block::max_exit_speed()
{
    if(is_ticking) return this->exit_speed;
    if (nominal_length_flag) return nominal_speed;
    float max = max_allowable_speed(-this->acceleration, this->entry_speed, this->millimeters);
    return min(max, nominal_speed);
}

void Block::prepare(float acceleration_in_steps, float deceleration_in_steps)
{
    float inv = 1.0F / this->steps_event_count;
    double acceleration_per_tick = acceleration_in_steps * fp_scale;
    double deceleration_per_tick = deceleration_in_steps * fp_scale;

    for (uint8_t m = 0; m < n_actuators; m++) {
        uint32_t steps = this->steps[m];
        this->tick_info[m].steps_to_move = steps;
        this->tick_info[m].jerk_change = 0;
        this->tick_info[m].accel_jerk_change = 0;
        this->tick_info[m].decel_jerk_change = 0;
        if(steps == 0) continue;

        float aratio = inv * steps;
        this->tick_info[m].steps_per_tick = (int64_t)round((((double)this->initial_rate * aratio) / STEP_TICKER_FREQUENCY) * STEPTICKER_FPSCALE);
        this->tick_info[m].counter = 0;
        this->tick_info[m].step_count = 0;
        this->tick_info[m].next_accel_event = this->total_move_ticks + 1;

        double acceleration_change = 0;
        if(this->accelerate_until != 0) {
            this->tick_info[m].next_accel_event = this->accelerate_until;
            acceleration_change = acceleration_per_tick;
        } else if(this->decelerate_after == 0) {
            acceleration_change = -deceleration_per_tick;
        } else if(this->decelerate_after != this->total_move_ticks) {
            this->tick_info[m].next_accel_event = this->decelerate_after;
        }

        this->tick_info[m].acceleration_change= (int64_t)round(acceleration_change * aratio);
        this->tick_info[m].deceleration_change= -(int64_t)round(deceleration_per_tick * aratio);
        this->tick_info[m].plateau_rate= (int64_t)round(((this->maximum_rate * aratio) / STEP_TICKER_FREQUENCY) * STEPTICKER_FPSCALE);
    }
}

void Block::prepare_s_curve(float accel_jerk_steps_s3, float decel_jerk_steps_s3)
{
    const double frequency = STEP_TICKER_FREQUENCY;
    const double qscale = (double)STEPTICKER_FPSCALE;
    const double inv_f3 = 1.0 / (frequency * frequency * frequency);
    const float inv = 1.0F / this->steps_event_count;

    for(uint8_t m = 0; m < n_actuators; ++m) {
        const uint32_t steps = this->steps[m];
        tick_info[m].steps_to_move = steps;
        tick_info[m].counter = 0;
        tick_info[m].step_count = 0;
        tick_info[m].next_accel_event = total_move_ticks + 1;
        tick_info[m].acceleration_change = 0;
        tick_info[m].deceleration_change = 0;
        tick_info[m].jerk_change = 0;
        tick_info[m].accel_jerk_change = 0;
        tick_info[m].decel_jerk_change = 0;
        tick_info[m].plateau_rate = 0;
        if(steps == 0) continue;

        const double ratio = (double)inv * (double)steps;
        tick_info[m].steps_per_tick = (int64_t)llround((((double)initial_rate * ratio) / frequency) * qscale);
        tick_info[m].plateau_rate = (int64_t)llround((((double)maximum_rate * ratio) / frequency) * qscale);

        const int64_t ja = (int64_t)llround((double)accel_jerk_steps_s3 * ratio * inv_f3 * qscale);
        const int64_t jd = (int64_t)llround((double)decel_jerk_steps_s3 * ratio * inv_f3 * qscale);
        tick_info[m].accel_jerk_change = ja;
        tick_info[m].decel_jerk_change = jd;
        tick_info[m].jerk_change = ja;
    }
}

float Block::get_trapezoid_rate(int i) const
{
    return STEPTICKER_FROMFP(tick_info[i].steps_per_tick) * STEP_TICKER_FREQUENCY;
}
