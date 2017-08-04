/*
      This file is part of Smoothie (http://smoothieware.org/). The motion
      control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under
      the terms of the GNU General Public License as published by the Free
      Software Foundation, either version 3 of the License, or (at your option)
      any later version.  Smoothie is distributed in the hope that it will be
      useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
      Public License for more details.  You should have received a copy of the
      GNU General Public License along with Smoothie. If not, see
      <http://www.gnu.org/licenses/>.
*/
#include "StepperMotor.h"

#include "Kernel.h"
#include "MRI_Hooks.h"
#include "StepTicker.h"

#include <math.h>
#include "mbed.h"

#include "constants.h"

StepperMotor::StepperMotor(Pin &step, Pin &dir, Pin &en) : step_pin(step), dir_pin(dir), en_pin(en)
{
    set_high_on_debug(en.port_number, en.pin);

    X = QV = QA = QVt = 0;
    s = L = L1 = QV1 = 0;

    moving= false;

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
    this->X += (dir ? 1 : -1);
}

void StepperMotor::set_speed(int64_t speed) {
    if (!is_enabled()) enable(true);
    moving = true;

    QVt = speed;
    updateQA();
}

void StepperMotor::zero_position() {
    X = 0;
}

bool StepperMotor::will_crash() {
//    return 10*QV*QV >= 30*(is_emptying()?(Xmax-X):X)*Q*QAmax;
    return is_emptying()?(Xmax<=X):(X<=0);
}

void StepperMotor::updateQA() {
    QA = QVt - QV;
    if (QA > QAmax) {
        QA = QAmax;
    } else if (QA < -QAmax) {
        QA = -QAmax;
    }
}

// TODO change direction if necessary
// returns whether or not we stepped the motor
bool StepperMotor::tick() {
    s ++;
    QV1 += QA;
    L = L1;
//    L1 += 2*QV1 - QV;
    L1 = s * QV;
//    if (abs(L) < 2*Q && abs(L1) >= 2*Q) {
    if (abs(L) < Q && abs(L1) >= Q) {
        set_direction(L1 > 0);
        step();
        updateQA();
        s = L = L1 = 0;
        return true;
    }
    return false;
}
