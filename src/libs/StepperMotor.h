/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "Module.h"
#include "Pin.h"

class StepperMotor  : public Module {
    public:
        StepperMotor(Pin& step, Pin& dir, Pin& en);
        ~StepperMotor();

        void set_motor_id(uint8_t id) { motor_id= id; }
        uint8_t get_motor_id() const { return motor_id; }

        // called from step ticker ISR
        inline bool step() { step_pin.set(1); current_position_steps += (direction?-1:1); return moving; }
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
        void set_speed(float speed);
        int get_speed() const { return target_delta * (target_dir ? 1 : -1); };
        int get_actual_speed() const { return tick_delta * (direction ? 1 : -1); };

        inline bool which_direction() const { return direction; }

        float get_steps_per_second()  const { return steps_per_second; }
        float get_steps_per_mm()  const { return steps_per_mm; }
        void change_steps_per_mm(float);
        float get_current_position(void) const { return (float)current_position_steps/steps_per_mm; }
        uint32_t get_current_step(void) const { return current_position_steps; }

        inline float get_current_speed(void) const { return current_speed; }
        inline void set_current_speed(float cs) { current_speed = cs; }

        float get_max_rate(void) const { return max_rate; }
        void set_max_rate(float mr) { max_rate= mr; }
        bool is_selected() const { return selected; }
        void set_selected(bool b) { selected= b; }

        unsigned int tick_delta = 100000;
        unsigned int target_delta = 100000;
        volatile unsigned int last_tick = 0;
        bool target_dir = 1;
        volatile float target_speed = 0;
        float now_speed = 0;

        void zero_position();
        float stop_dist();
        bool will_crash();
        float time_to_fill();
        float time_to_empty();

        volatile float current_speed;

    private:
        void on_halt(void *argument);
        void on_enable(void *argument);

        Pin step_pin;
        Pin dir_pin;
        Pin en_pin;

        float steps_per_second;
        float steps_per_mm;
        float max_rate; // this is not really rate it is in mm/sec, misnamed used in Robot and Extruder

        volatile int32_t current_position_steps;

        volatile struct {
            uint8_t motor_id:8;
            volatile bool direction:1;
            volatile bool moving:1;
            bool selected:1;
        };
};

