
#include "Heater.h"

#include "mbed.h"
#include "system_LPC17xx.h" // mbed.h lib

Heater *Heater::instance;
bool Heater::heat_enabled = false;

Heater::Heater() {
    /*
    instance = this; // setup the Singleton instance of the heater

    this->ZeroPin.from_string("0.27");
    this->ZeroPin.as_input();
    this->ZeroPin.pull_up();
    this->HeaterPin.from_string("0.28");
    this->HeaterPin.as_output();
    this->HeaterPin.set(0);

    LPC_SC->PCONP |= (1<<23); // Enable Timer 3

    set_delay_us(6000); // set delay to 6000us by default

    LPC_TIM3->PR = 0;
    LPC_TIM3->CTCR = 0;

    LPC_TIM3->MCR = 5; // on match, interrupt and stop
//    LPC_TIM3->MCR = 3; // on match, interrupt and stop
    LPC_TIM3->TCR = 0; // disable interrupt

    this->ZeroPin.interrupt_pin()->rise(Heater::ip_rose);
    NVIC_EnableIRQ(TIMER3_IRQn);    // Enable interrupt handler
    */
}

void
Heater::set_delay_us(int microseconds) {
    LPC_TIM3->MR0 = floorf((SystemCoreClock / 4.0F) * (microseconds / 1000000.0F));
}

void Heater::ip_rose() {
    Heater::getInstance()->heater_phase_wait();

    if (heat_enabled) {
        LPC_TIM3->TCR = 3;  // Reset
        LPC_TIM3->TCR = 1;  // Reset
    }
}

void Heater::enable(bool enable_flag) {
    heat_enabled = enable_flag;
}

extern "C" void TIMER3_IRQHandler (void) {
    LPC_TIM3->IR |= 1 << 0;          // Clear MR0 interrupt flag
    Heater::getInstance()->heater_phase_start();
}
