/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <stdint.h>

#include "Module.h"
#include "Pin.h"

class StepperMotor  : public Module {
    public:
        StepperMotor(Pin& step, Pin& dir, Pin& en);
        ~StepperMotor();

        void set_motor_id(uint8_t id) { motor_id= id; }
        uint8_t get_motor_id() const { return motor_id; }

        // called from step ticker ISR
        inline bool step() { step_pin.set(1); X += (direction?1:-1); return moving; }
        // called from unstep ISR
        inline void unstep() { step_pin.set(0); }
        // called from step ticker ISR
        inline void set_direction(bool f) { dir_pin.set(f); direction= f; }
        bool get_direction() const { return direction; }

        void enable(bool state) { en_pin.set(!state); };
        bool is_enabled() const { return !en_pin.get(); };
        inline bool is_moving() const { return moving; };
        void start_moving() { moving= true; };
        void stop_moving() { moving= false; };

        void manual_step(bool dir);
        void set_speed(int64_t speed);
        int64_t get_speed() const { return QVt; };
        int64_t get_actual_speed() const { return QV; };

        inline bool which_direction() const { return direction; }

        int64_t get_current_step(void) const { return X; }

        inline bool is_emptying() { return QVt > 0; }
        inline bool is_filling() { return QVt <= 0; }

        void zero_position();

        volatile int64_t X;
        volatile int64_t QV, QVt; // actual and target velocities
        volatile int64_t QA;
        int64_t s, L, L1, QV1;

        bool tick();
        bool will_crash();

    private:
        void on_halt(void *argument);
        void on_enable(void *argument);
        void updateQA();

        Pin step_pin;
        Pin dir_pin;
        Pin en_pin;

        volatile struct {
            uint8_t motor_id:8;
            volatile bool direction:1;
            volatile bool moving:1;
        };
};
