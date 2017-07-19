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

#include "TSRingBuffer.h"

class StepperMotor;

// handle 2.30 Fixed point
#define STEPTICKER_FPSCALE (1<<30)
#define STEPTICKER_TOFP(x) ((int32_t)roundf((float)(x)*STEPTICKER_FPSCALE))
#define STEPTICKER_FROMFP(x) ((float)(x)/STEPTICKER_FPSCALE)

#define k_max_actuators 6

class StepTicker{
    public:
        StepTicker();
        ~StepTicker();
        void set_frequency( float frequency );
        void set_unstep_time( float microseconds );
        int register_motor(StepperMotor* motor);
        float get_frequency() const { return frequency; }
        void unstep_tick();

        void step_tick (void);
        void handle_finish (void);
        void start();

        // whatever setup the block should register this to know when it is done
        std::function<void()> finished_fnc{nullptr};

        static StepTicker *getInstance() { return instance; }

        void manual_step(int i, bool dir);
        void set_speed(int i, int speed);

        int get_num_motors() const { return num_motors; };
        StepperMotor* get_motor(int mi) const { return motor[mi]; };

        int get_actual_speed(int i) const;
        int get_current_step(int i) const;
        int get_current_speed(int i) const;

        void stop();

        void pump_speed(int speed);
        bool is_pumping() const { return pumping; };
        int get_pump_speed();
        void zero_motors();

        uint32_t on_speed_up(uint32_t dummy);
        uint32_t on_speed_down(uint32_t dummy);

    private:
        static StepTicker *instance;

        float frequency;
        uint32_t period;
        float flux_hat;
        std::array<StepperMotor*, k_max_actuators> motor;
        std::bitset<k_max_actuators> unstep;
        std::bitset<k_max_actuators> reverse;

        uint32_t current_tick{0};

        struct {
            volatile bool running:1;
            volatile bool pumping:1;
            volatile bool zeroing:1;
            uint8_t num_motors:4;
        };
};
