/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/
#include "StepperMotor.h"

#include "Kernel.h"
#include "MRI_Hooks.h"
#include "StepTicker.h"

#include <math.h>
#include "mbed.h"

#define a_max (10.)
#define v_max (20000.)
#define l_steps (25000.)

StepperMotor::StepperMotor(Pin &step, Pin &dir, Pin &en) : step_pin(step), dir_pin(dir), en_pin(en)
{
    set_high_on_debug(en.port_number, en.pin);

    steps_per_mm         = 1.0F;
    max_rate             = 50.0F;

    current_position_steps= 0;
    current_speed = 0.0F;
    target_speed = 0.0F;

    moving= false;
    selected= true;

    enable(false);
    unstep(); // initialize step pin
    set_direction(false); // initialize dor pin

    this->register_for_event(ON_HALT);
    this->register_for_event(ON_ENABLE);
}

StepperMotor::~StepperMotor()
{
    THEKERNEL->unregister_for_event(ON_HALT, this);
    THEKERNEL->unregister_for_event(ON_ENABLE, this);
}

void StepperMotor::on_halt(void *argument)
{
    if(argument == nullptr) {
        enable(false);
        moving= false;
    }
}

void StepperMotor::on_enable(void *argument)
{
    // argument is a uin32_t where bit0 is on or off, and bit 1:X, 2:Y, 3:Z, 4:A, 5:B, 6:C etc
    // for now if bit0 is 1 we turn all on, if 0 we turn all off otherwise we turn selected axis off
    uint32_t bm= (uint32_t)argument;
    if(bm == 0x01) {
        enable(true);

    }else if(bm == 0 || ((bm&0x01) == 0 && ((bm&(0x02<<motor_id)) != 0)) ) {
        enable(false);
    }
}

void StepperMotor::change_steps_per_mm(float new_steps)
{
    steps_per_mm = new_steps;
}

// Does a manual step pulse, used for direct encoder control of a stepper
// NOTE this is experimental and may change and/or be reomved in the future, it is an unsupported feature.
// use at your own risk
void StepperMotor::manual_step(bool dir) {
    if(!is_enabled()) enable(true);

    // set direction if needed
    if(this->direction != dir) {
        this->direction= dir;
        this->dir_pin.set(dir);
        wait_us(1);
    }

    // pulse step pin
    this->step_pin.set(1);
    wait_us(3);
    this->step_pin.set(0);


    // keep track of actuators actual position in steps
    this->current_position_steps += (dir ? -1 : 1);
}

void StepperMotor::set_speed(float speed) {
    if (!is_enabled()) enable(true);
    moving = true;

    target_speed = speed;
}

void StepperMotor::zero_position() {
    current_position_steps = 0;
}

float stop_dist_raw(float v) {
    return 1.;
    return 0.5 * v * v / a_max;
}

float StepperMotor::stop_dist() {
    return stop_dist_raw(current_speed);
}

bool StepperMotor::will_crash() {
    if (direction) {
        if (current_position_steps <= -l_steps) {
            return true;
        }
        return current_speed > 0 && stop_dist() - current_position_steps >= l_steps;
    } else {
        if (current_position_steps >= 0) {
            return true;
        }
        return current_speed < 0 && stop_dist() >= -current_position_steps;
    }
}

float time_to_fill_raw(float x, float v) {
    return x / v_max;

    float rval = 0;
    float half_time = 0;
    float v2;

    if (v > 0) {
        rval += v / a_max;
        x += stop_dist_raw(v);
        v = 0;
    }

    rval += v / a_max;
    x += 0.5 * v * v / a_max;
    v = 0;

    half_time = sqrt(x / a_max);
    v2 = half_time * a_max;

    if (v2 < v_max) {
        rval += 2 * half_time;
    } else {
        rval += (x - 2 * v_max * v_max / a_max) / v_max + 2 * (v_max / a_max);
    }

    v2 = v_max;
    rval += x / v_max;

    return rval;
}

float StepperMotor::time_to_fill() {
    return time_to_fill_raw(-current_position_steps, current_speed);
}

float StepperMotor::time_to_empty() {
    return time_to_fill_raw(l_steps + current_position_steps, -current_speed);
}
