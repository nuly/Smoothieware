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


#include "StepTicker.h"

#include "libs/nuts_bolts.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "StepperMotor.h"
#include "StreamOutputPool.h"
#include "modules/encoder/RotaryEncoder.h"

#include "system_LPC17xx.h" // mbed.h lib
#include <math.h>
#include <mri.h>

#include "int128.h"
#include "constants.h"

#ifdef STEPTICKER_DEBUG_PIN
// debug pins, only used if defined in src/makefile
#include "gpio.h"
GPIO stepticker_debug_pin(STEPTICKER_DEBUG_PIN);
#define SET_STEPTICKER_DEBUG_PIN(n) {if(n) stepticker_debug_pin.set(); else stepticker_debug_pin.clear(); }
#else
#define SET_STEPTICKER_DEBUG_PIN(n)
#endif

StepTicker *StepTicker::instance;

StepTicker::StepTicker()
{
    instance = this; // setup the Singleton instance of the stepticker

    // Configure the timer
    LPC_TIM0->MR0 = 10000000;       // Initial dummy value for Match Register
    LPC_TIM0->MCR = 3;              // Match on MR0, reset on MR0
    LPC_TIM0->TCR = 0;              // Disable interrupt

    LPC_SC->PCONP |= (1 << 2);      // Power Ticker ON
    LPC_TIM1->MR0 = 1000000;
    LPC_TIM1->MCR = 5;              // match on Mr0, stop on match
    LPC_TIM1->TCR = 0;              // Disable interrupt

    // Default start values
    this->set_frequency(1000);
    this->set_unstep_time(100);

    this->reverse.reset();
    this->unstep.reset();
    this->num_motors = 0;

    this->running = false;
    this->pumping = false;
    this->zeroing = false;

    this->flux_hat = 0;

    #ifdef STEPTICKER_DEBUG_PIN
    // setup debug pin if defined
    stepticker_debug_pin.output();
    stepticker_debug_pin= 0;
    #endif
}

StepTicker::~StepTicker()
{
}

//called when everything is setup and interrupts can start
void StepTicker::start()
{
    NVIC_EnableIRQ(TIMER0_IRQn);     // Enable interrupt handler
    NVIC_EnableIRQ(TIMER1_IRQn);     // Enable interrupt handler
    current_tick= 0;

#ifdef ROTARY_ENCODER_H
    RotaryEncoder::instance->up_attach(this, &StepTicker::on_speed_up);
    RotaryEncoder::instance->down_attach(this, &StepTicker::on_speed_down);
#endif
}

// Set the base stepping frequency
void StepTicker::set_frequency( float frequency )
{
    this->frequency = frequency;
    this->period = floorf((SystemCoreClock / 4.0F) / frequency); // SystemCoreClock/4 = Timer increments in a second
    LPC_TIM0->MR0 = this->period;
    LPC_TIM0->TCR = 3;  // Reset
    LPC_TIM0->TCR = 1;  // start
}

// Set the reset delay, must be called after set_frequency
void StepTicker::set_unstep_time( float microseconds )
{
    uint32_t delay = floorf((SystemCoreClock / 4.0F) * (microseconds / 1000000.0F)); // SystemCoreClock/4 = Timer increments in a second
    LPC_TIM1->MR0 = delay;

    // TODO check that the unstep time is less than the step period, if not slow down step ticker
}

// Reset step pins on any motor that was stepped
void StepTicker::unstep_tick()
{
    for (int i = 0; i < num_motors; i++) {
        if(this->unstep[i]) {
            this->motor[i]->unstep();
        }
    }
    this->unstep.reset();
}

extern "C" void TIMER1_IRQHandler (void)
{
    LPC_TIM1->IR |= 1 << 0;
    StepTicker::getInstance()->unstep_tick();
}

// The actual interrupt handler where we do all the work
extern "C" void TIMER0_IRQHandler (void)
{
    // Reset interrupt register
    LPC_TIM0->IR |= 1 << 0;
    StepTicker::getInstance()->step_tick();
}

extern "C" void PendSV_Handler(void)
{
    StepTicker::getInstance()->handle_finish();
}

// slightly lower priority than TIMER0, the whole end of block/start of block is done here allowing the timer to continue ticking
void StepTicker::handle_finish (void)
{
    // all moves finished signal block is finished
    if(finished_fnc) finished_fnc();
}

#define sq(x) ((x)*(x))

bool
LongerToFill(StepperMotor* A, StepperMotor* B) {
    return 11*(A->X + B->X) > 10*Xmax;
}

/*
bool
LongerToFill(StepperMotor* A, StepperMotor* B) {
    uint64_t sigma1, sigma2, tmp;
    int sgn2;
    uint64_t QVX128 = QVmax;
    sigma1 = A->X * Q * QAmax + (A->QV)*(A->QV)/2;

//    sigma2 = (Xmax - B->X) * Q * QAmax - (B->QV)*(B->QV)/2;
    sigma2 = (Xmax - B->X) * Q * QAmax;
    tmp = (B->QV)*(B->QV)/2;
    if (sigma2 < tmp) {
        sgn2 = -1;
        sigma2 = tmp - sigma2;
    } else {
        sgn2 = 1;
        sigma2 -= tmp;
    }
    if (sigma1 >= sq(QVX128) && sgn2 > 0 && sigma2 >= sq(QVX128)) {
        return (sigma1 + QVX128 * A->QV) >= (sigma2 - QVX128 * B->QV);
    } else if (sigma1 < sq(QVX128) && sgn2 > 0 && sigma2 >= sq(QVX128)) {
        // we will approximate by dividing through by sq(QVX128)
        return sigma2 + sq(QVX128) <= QVX128 * (A->QV + B->QV) ||
//            sigma1*sq(QVX128)*4 >= sq(sigma2+sq(QVX128)-QVX128*B->QV-QVX128*A->QV);
            sigma1*4 >= sq(sigma2/QVX128+QVX128-B->QV-A->QV);
    } else if (sigma1 >= sq(QVX128) && (sgn2 < 0 || sigma2 < sq(QVX128))) {
        return (A->QV + B->QV < 0 && sigma1 + sq(QVX128) <= QVX128 * (-A->QV - B->QV)) ||
            (sgn2 > 0 && 
//            sigma2*sq(QVX128)*4 >= sq(sigma1+sq(QVX128)+QVX128*B->QV+QVX128*A->QV));
            sigma2*4 >= sq(sigma1/QVX128+QVX128+B->QV+A->QV));
    } else {
        tmp = sq(A->QV + B->QV);
        if (A->QV + B->QV < 0 && sigma1 <= tmp) {
            return false;
        } else if (sgn2 < 0 || sigma2*4 <= sigma1*4 + tmp) {
            return A->QV + B->QV >= 0;
        } else {
            // sgn2 > 0 for sure
            return (sigma1+sigma2)*tmp*8 >= sq(sigma2*4-sigma1*4) + sq(tmp);
        }
    }
}
*/

// step clock
void StepTicker::step_tick (void)
{
    //SET_STEPTICKER_DEBUG_PIN(running ? 1 : 0);

    if(THEKERNEL->is_halted()) {
        running= false;
        current_tick = 0;
        return;
    }

    if (pumping) {
        reverse.reset();
        for (uint8_t m = 0; m < NUM_PUMPING; m++) {
            if (motor[m]->is_emptying()) {
                uint8_t mnext = (m+NUM_PUMPING-1) % NUM_PUMPING;

                if (motor[m]->will_crash() || 
                        (motor[mnext]->is_emptying() &&
                         LongerToFill(motor[m], motor[mnext]))) {
                    reverse.set(m);
                }
            }
        }

        for (uint8_t m = 0; m < NUM_PUMPING; m++) {
            if (reverse[m]) {
                uint8_t mprev = (m+1)%NUM_PUMPING;
                motor[m]->set_speed(-QVmax);
                motor[mprev]->set_speed(flux_hat/NUM_FORWARD);
            }
        }

        for (uint8_t m = 0; m < NUM_PUMPING; m++) {
            if (motor[m]->is_filling() && motor[m]->will_crash()) {
                motor[m]->set_speed(motor[m]->get_speed()/2);
            }
        }
    } else if (zeroing) {
        for (uint8_t m = 0; m < num_motors; m++) {
            if (motor[m]->will_crash()) {
                motor[m]->set_speed(motor[m]->get_speed()/2);
            }
        }
    }

    // for each motor
    for (uint8_t m = 0; m < num_motors; m++) {
        if (!motor[m]->is_moving()) {
            continue;
        }

        if (motor[m]->tick()) {
            unstep.set(m);
        }
    }

    // do this after so we start at tick 0
    current_tick++; // count number of ticks

    // We may have set a pin on in this tick, now we reset the timer to set it off
    // Note there could be a race here if we run another tick before the unsteps have happened,
    // right now it takes about 3-4us but if the unstep were near 10uS or greater it would be an issue
    // also it takes at least 2us to get here so even when set to 1us pulse width it will still be about 3us
    if( unstep.any()) {
        LPC_TIM1->TCR = 3;
        LPC_TIM1->TCR = 1;
    }
}

// returns index of the stepper motor in the array and bitset
int StepTicker::register_motor(StepperMotor* m)
{
    motor[num_motors++] = m;
    return num_motors - 1;
}

void StepTicker::manual_step(int i, bool dir) {
    if (i >= 0 && i < num_motors) {
        motor[i]->manual_step(dir);
    }
}

void StepTicker::set_speed(int i, int64_t speed) {
    if (i >= 0 && i < num_motors) {
        motor[i]->set_speed(speed);
    }
}

uint32_t StepTicker::on_speed_up(uint32_t dummy) {
    if (flux_hat + QVDELTA > QVmax) {
        flux_hat = QVmax;
    } else {
        flux_hat += QVDELTA;
    }
    this->pump_speed(flux_hat);
    return 0;
}

uint32_t StepTicker::on_speed_down(uint32_t dummy) {
    if (flux_hat - QVDELTA < 0) {
        flux_hat = 0;
    } else {
        flux_hat -= QVDELTA;
    }
    this->pump_speed(flux_hat);
    return 0;
}

int64_t StepTicker::get_actual_speed(int i) const { return motor[i]->QV; }
int64_t StepTicker::get_current_step(int i) const { return motor[i]->X; }
int64_t StepTicker::get_current_speed(int i) const { return motor[i]->QVt; }

void StepTicker::stop() {
    pumping = false;
    zeroing = true;
    for (uint8_t m = 0; m < num_motors; m++) {
        motor[m]->stop_moving();
    }
}

void StepTicker::pump_speed(int64_t speed) {
    flux_hat = speed;

    if (flux_hat > 0) {
        if (!pumping) {
            pumping = true;
            for (uint8_t m=0; m<NUM_PUMPING; m++) {
                motor[m]->set_speed(m<NUM_FORWARD ? (flux_hat)/NUM_FORWARD : -QVmax);
            }
        } else {
            for (uint8_t m=0; m<NUM_PUMPING; m++) {
                if (motor[m]->is_emptying()) {
                    motor[m]->set_speed((flux_hat)/NUM_FORWARD);
                }
            }
        }
    } else {
        // retract
        pumping = false;
        zeroing = true;
        for (uint8_t m=0; m<NUM_PUMPING; m++) {
            motor[m]->set_speed(-QVmax);
        }
    }
}

int64_t StepTicker::get_pump_speed() {
    float rval = 0;
    for (uint8_t m = 0; m < num_motors; m++) {
        if (motor[m]->is_moving() && get_current_speed(m) > 0) {
            rval += get_current_speed(m);
        }
    }
    return rval;
}

void StepTicker::zero_motors() {
    for (uint8_t m=0; m<num_motors; m++) {
        motor[m]->zero_position();
    }
}
