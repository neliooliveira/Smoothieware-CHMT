/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "StepTicker.h"

#include "libs/nuts_bolts.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "StepperMotor.h"
#include "StreamOutputPool.h"
#include "Block.h"
#include "Conveyor.h"
#include "modules/tools/flybyvision/FlyByVision.h"

#include "stm32f407xx.h"
#include <math.h>
#include <mri.h>

#ifdef STEPTICKER_DEBUG_PIN
#include "gpio.h"
GPIO stepticker_debug_pin(STEPTICKER_DEBUG_PIN);
#define SET_STEPTICKER_DEBUG_PIN(n) {if(n) stepticker_debug_pin.set(); else stepticker_debug_pin.clear(); }
#else
#define SET_STEPTICKER_DEBUG_PIN(n)
#endif

#define TIM7_PRESCALER      15
#define TIM14_PRESCALER     1

extern "C" void TIM8_TRG_COM_TIM14_IRQHandler(void);
extern "C" void TIM7_IRQHandler(void);
extern "C" void PendSV_Handler(void);

StepTicker *StepTicker::instance;

StepTicker::StepTicker()
{
    instance = this;

    __TIM7_CLK_ENABLE();
    TIM7->CR1 = TIM_CR1_URS;
    NVIC_SetVector(TIM7_IRQn, (uint32_t)TIM7_IRQHandler);

    __TIM14_CLK_ENABLE();
    TIM14->CR1 = TIM_CR1_URS | TIM_CR1_OPM;
    NVIC_SetVector(TIM8_TRG_COM_TIM14_IRQn, (uint32_t)TIM8_TRG_COM_TIM14_IRQHandler);

    NVIC_SetVector(PendSV_IRQn, (uint32_t)PendSV_Handler);

    this->set_frequency(200000);
    this->set_unstep_time(2);

    this->unstep.reset();
    this->num_motors = 0;
    this->running = false;
    this->current_block = nullptr;

    #ifdef STEPTICKER_DEBUG_PIN
    stepticker_debug_pin.output();
    stepticker_debug_pin= 0;
    #endif
}

StepTicker::~StepTicker()
{
}

void StepTicker::start()
{
    TIM7->DIER = TIM_DIER_UIE;
    TIM14->DIER = TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM7_IRQn);
    NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);
    NVIC_EnableIRQ(PendSV_IRQn);

    current_tick= 0;
    TIM7->CR1 |= TIM_CR1_CEN;
}

void StepTicker::set_frequency( float frequency )
{
    this->frequency = frequency;
    this->period = floorf((SystemCoreClock >> 1) / TIM7_PRESCALER / frequency);

    TIM7->PSC = TIM7_PRESCALER-1;
    TIM7->ARR = this->period;
}

void StepTicker::set_unstep_time( float microseconds )
{
    uint32_t delay = floorf(((SystemCoreClock >> 1) / TIM14_PRESCALER) * (microseconds / 1000000.0F));
    TIM14->ARR = delay;
}

void StepTicker::unstep_tick()
{
    for (int i = 0; i < num_motors; i++) {
        if(this->unstep[i]) {
            this->motor[i]->unstep();
        }
    }
    this->unstep.reset();
}

extern "C" void TIM8_TRG_COM_TIM14_IRQHandler (void)
{
    TIM14->SR = ~TIM_SR_UIF;
    StepTicker::getInstance()->unstep_tick();
}

extern "C" void TIM7_IRQHandler (void)
{
    TIM7->SR = ~TIM_SR_UIF;
    SET_STEPTICKER_DEBUG_PIN(1);
    StepTicker::getInstance()->step_tick();
    SET_STEPTICKER_DEBUG_PIN(0);
}

extern "C" void PendSV_Handler(void)
{
    StepTicker::getInstance()->handle_finish();
}

void StepTicker::handle_finish (void)
{
    if(finished_fnc) finished_fnc();
}

void StepTicker::step_tick (void)
{
    FlyByVision::service_pulses_from_isr();

    if(!running){
        if(THECONVEYOR->get_next_block(&current_block)) {
            running= start_next_block();
            if(!running) return;
        }else{
            return;
        }
    }

    if(THEKERNEL->is_halted()) {
        running= false;
        current_tick = 0;
        current_block= nullptr;
        return;
    }

    if(current_block->flyby_trigger.enabled() &&
       current_tick == current_block->flyby_trigger.trigger_tick) {
        if(flyby_hook != nullptr) {
            flyby_hook(current_block->flyby_trigger);
        }
        current_block->flyby_trigger.flags &= ~FlyByTrigger::ENABLED;
    }

    const bool s_curve = current_block->s_curve_active;
    bool still_moving= false;

    for (uint8_t m = 0; m < num_motors; m++) {
        if(current_block->tick_info[m].steps_to_move == 0) continue;

        Block::tickinfo_t& ti = current_block->tick_info[m];

        if(s_curve) {
            // Seven-phase jerk-limited profile. All values are fixed-point and
            // all boundary values were precomputed by the planner.
            if(current_tick == current_block->s_curve_phase_end[0]) {
                // +J -> constant +A
                ti.jerk_change = 0;
            } else if(current_tick == current_block->s_curve_phase_end[1]) {
                // constant +A -> -J, bringing acceleration back to zero
                ti.jerk_change = -ti.accel_jerk_change;
            } else if(current_tick == current_block->s_curve_phase_end[2]) {
                // acceleration complete -> cruise
                ti.jerk_change = 0;
                ti.acceleration_change = 0;
                ti.steps_per_tick = ti.plateau_rate;
            } else if(current_tick == current_block->s_curve_phase_end[3]) {
                // cruise -> -J, start deceleration
                ti.jerk_change = -ti.decel_jerk_change;
            } else if(current_tick == current_block->s_curve_phase_end[4]) {
                // -J -> constant -A
                ti.jerk_change = 0;
            } else if(current_tick == current_block->s_curve_phase_end[5]) {
                // constant -A -> +J, return acceleration to zero
                ti.jerk_change = ti.decel_jerk_change;
            } else if(current_tick == current_block->s_curve_phase_end[6]) {
                ti.jerk_change = 0;
                ti.acceleration_change = 0;
            }

            ti.acceleration_change += ti.jerk_change;
            ti.steps_per_tick += ti.acceleration_change;

        } else {
            ti.steps_per_tick += ti.acceleration_change;

            if(current_tick == ti.next_accel_event) {
                if(current_tick == current_block->accelerate_until) {
                    ti.acceleration_change = 0;
                    if(current_block->decelerate_after < current_block->total_move_ticks) {
                        ti.next_accel_event = current_block->decelerate_after;
                        if(current_tick != current_block->decelerate_after) {
                            ti.steps_per_tick = ti.plateau_rate;
                        }
                    }
                }

                if(current_tick == current_block->decelerate_after) {
                    ti.acceleration_change = ti.deceleration_change;
                }
            }
        }

        if(ti.steps_per_tick <= 0) {
            ti.counter = STEPTICKER_FPSCALE;
            ti.steps_per_tick = 0;
        }

        ti.counter += ti.steps_per_tick;

        if(ti.counter >= STEPTICKER_FPSCALE) {
            ti.counter -= STEPTICKER_FPSCALE;
            ++ti.step_count;

            bool ismoving= motor[m]->step();
            unstep.set(m);

            if(!ismoving || (!motor[m]->is_encoder_controlled() && ti.step_count == ti.steps_to_move)) {
                ti.steps_to_move = 0;
                motor[m]->stop_moving();
            }
        }

        if(motor[m]->is_moving()) still_moving= true;
    }

    current_tick++;

    if(unstep.any()) {
        TIM14->CR1 |= TIM_CR1_CEN;
    }

    if(!still_moving) {
        current_tick = 0;
        current_block->flyby_trigger.clear();

        THECONVEYOR->block_finished();

        if(THECONVEYOR->get_next_block(&current_block)) {
            running= start_next_block();
        }else{
            current_block= nullptr;
            running= false;
        }

        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    }
}

bool StepTicker::start_next_block()
{
    if(current_block == nullptr) return false;

    bool ok= false;
    for (uint8_t m = 0; m < num_motors; m++) {
        if(current_block->tick_info[m].steps_to_move == 0) continue;

        ok= true;
        motor[m]->set_direction(current_block->direction_bits[m]);
        motor[m]->start_moving();
    }

    current_tick= 0;

    if(ok) {
        return true;
    }else{
        current_block->flyby_trigger.clear();
        THECONVEYOR->block_finished();
    }

    return false;
}

int StepTicker::register_motor(StepperMotor* m)
{
    motor[num_motors++] = m;
    return num_motors - 1;
}
