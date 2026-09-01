/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/



#pragma once

#include <stdint.h>
#include <array>
#include <bitset>
#include <functional>
#include <atomic>

#include "ActuatorCoordinates.h"
#include "TSRingBuffer.h"

class StepperMotor;
class Block;
struct FlyByTrigger;

// handle 2.62 Fixed point
#define STEPTICKER_FPSCALE (1LL<<62)
#define STEPTICKER_FROMFP(x) ((float)(x)/STEPTICKER_FPSCALE)

class StepTicker{
    public:
        // Optional ISR-safe observer for deterministic peripherals. The callback
        // executes from TIM7 and must not block, allocate, print, or use floats.
        using flyby_hook_t = void (*)(const FlyByTrigger& trigger);

        StepTicker();
        ~StepTicker();
        void set_frequency( float frequency );
        void set_unstep_time( float microseconds );
        int register_motor(StepperMotor* motor);
        float get_frequency() const { return frequency; }
        void unstep_tick();
        const Block *get_current_block() const { return current_block; }

        void step_tick (void);
        void handle_finish (void);
        void start();

        void set_flyby_hook(flyby_hook_t hook) { flyby_hook = hook; }

        // whatever setup the block should register this to know when it is done
        std::function<void()> finished_fnc{nullptr};

        static StepTicker *getInstance() { return instance; }

    private:
        static StepTicker *instance;

        bool start_next_block();

        float frequency;
        uint32_t period;
        std::array<StepperMotor*, k_max_actuators> motor;
        std::bitset<k_max_actuators> unstep;

        Block *current_block;
        uint32_t current_tick{0};
        flyby_hook_t flyby_hook{nullptr};

        struct {
            volatile bool running:1;
            uint8_t num_motors:4;
        };
};
