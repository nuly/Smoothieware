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

#include "system_LPC17xx.h" // mbed.h lib
#include <math.h>
#include <mri.h>

#ifdef STEPTICKER_DEBUG_PIN
// debug pins, only used if defined in src/makefile
#include "gpio.h"
GPIO stepticker_debug_pin(STEPTICKER_DEBUG_PIN);
#define SET_STEPTICKER_DEBUG_PIN(n) {if(n) stepticker_debug_pin.set(); else stepticker_debug_pin.clear(); }
#else
#define SET_STEPTICKER_DEBUG_PIN(n)
#endif

#define NUM_FORWARD (2)

#define a_max (10.)
#define v_max (20000.)
#define l_steps (25000.)
#define INF (v_max+1)

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
        for (uint8_t m = 0; m < num_motors; m++) {
            if (motor[m]->get_direction() && motor[m]->will_crash()) {
                reverse.set(m);
            }

            uint8_t mnext = (m+1) % num_motors;

            if (motor[m]->get_direction() && 
                motor[mnext]->get_direction() &&
                motor[mnext]->time_to_empty() <= motor[m]->time_to_fill()) {
                reverse.set(m);
            }
        }

        for (uint8_t m = 0; m < num_motors; m++) {
            if (reverse[m]) {
                uint8_t mprev = (m+num_motors-1)%num_motors;
                motor[m]->set_direction(false);
                motor[mprev]->set_direction(true);
            }
        }

        float flux_err = 0, vn;
        float rflux;
        for (int ig = 0; ig < 2; ig++) {
            rflux = 0;
            for (uint8_t m = 0; m < num_motors; m++) {
                if (motor[m]->get_direction()) {
                    vn = (flux_hat + flux_err) / ((float)NUM_FORWARD);
                } else if (motor[m]->will_crash()) {
                    vn = 0;
                } else {
                    vn = -INF;
                }

                /*
                if (vn - motor[m]->get_current_speed() > a_max) {
                    vn = motor[m]->get_current_speed() + a_max;
                } else if (vn - motor[m]->get_current_speed() < -a_max) {
                    vn = motor[m]->get_current_speed() - a_max;
                }
                */

                if (vn > v_max) {
                    vn = v_max;
                } else if (vn < -v_max) {
                    vn = -v_max;
                }

                if (ig == 1) {
                    motor[m]->set_current_speed(vn);
                }

                if (vn > 0) {
                    rflux += vn;
                }
            }

            flux_err = flux_hat - rflux;
        }
    } else {
        float vn, cs;
        for (uint8_t m = 0; m < num_motors; m++) {
            if (!motor[m]->is_moving()) continue;

            vn = motor[m]->target_speed;
            cs = motor[m]->current_speed;
            /*
            if (vn - cs > a_max) {
                vn = cs + a_max;
            } else if (vn - cs < -a_max) {
                vn = cs - a_max;
            }
            */
            if (vn > v_max) {
                vn = v_max;
            } else if (vn < -v_max) {
                vn = -v_max;
            }
            motor[m]->set_current_speed(vn);
        }
    }

    // for each motor
    for (uint8_t m = 0; m < num_motors; m++) {
        if (!motor[m]->is_moving()) {
            continue;
        }

        // get direction set up
        int td = current_tick - motor[m]->last_tick;
        if (td * abs(motor[m]->get_current_speed()) > frequency) {
            // time to tick!
            motor[m]->last_tick = current_tick;

            bool cdirection = motor[m]->get_current_speed() > 0;

            if (motor[m]->which_direction() ^ cdirection) {
                motor[m]->set_direction(cdirection);
            }

            motor[m]->step();
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

void StepTicker::set_speed(int i, float speed) {
    if (i >= 0 && i < num_motors) {
        motor[i]->set_speed(speed);
        motor[i]->last_tick = current_tick;
    }
}

int StepTicker::get_speed(int i) const { return motor[i]->get_speed(); }
int StepTicker::get_actual_speed(int i) const { return motor[i]->get_actual_speed(); }
int StepTicker::get_current_step(int i) const { return motor[i]->get_current_step(); }
int StepTicker::get_current_speed(int i) const { return motor[i]->get_current_speed(); }

void StepTicker::stop() {
    pumping = false;
    for (uint8_t m = 0; m < num_motors; m++) {
        motor[m]->stop_moving();
    }
}

void StepTicker::pump_speed(float speed) {
    flux_hat = speed;

    if (!pumping) {
        pumping = true;
        for (uint8_t m=0; m<num_motors; m++) {
            motor[m]->set_direction(m>=num_motors-NUM_FORWARD);
        }
    }
}

float StepTicker::get_pump_speed() {
    float rval = 0;
    for (uint8_t m = 0; m < num_motors; m++) {
        if (motor[m]->is_moving() && motor[m]->get_direction()) {
            rval += motor[m]->get_current_speed();
        }
    }
    return rval;
}


void StepTicker::zero_motors() {
    for (uint8_t m=0; m<num_motors; m++) {
        motor[m]->zero_position();
    }
}
