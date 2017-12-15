#include "Heater.h"

#include "libs/Kernel.h"

#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"

#include "mbed.h"
#include "system_LPC17xx.h" // mbed.h lib

#include "InterruptIn.h"

Heater *Heater::instance;
bool Heater::heat_enabled = false;
GPIO *Heater::led_to_blink = NULL;

int Heater::count = 0;

Heater::Heater(GPIO *led_to_blink = NULL) {
    instance = this; // setup the Singleton instance of the heater

    this->led_to_blink = led_to_blink;

    this->ZeroPin.from_string(
                    THEKERNEL->config->value(CHECKSUM("zero_pin"))
                    ->by_default("2.11")->as_string()
                    )
            ->as_input();
//    ->pull_up();

    this->HeaterPin.from_string(
                    THEKERNEL->config->value(CHECKSUM("heater0_pin"))
                    ->by_default("3.26")->as_string()
                    )
            ->as_output();
    this->HeaterPin.set(0);

    LPC_SC->PCONP |= (1<<23); // Enable Timer 3

    set_delay_us(9000); // set delay to 9000us by default

    LPC_TIM3->PR = 0;
    LPC_TIM3->CTCR = 0;

    LPC_TIM3->MCR = 5; // on match, interrupt and stop
//    LPC_TIM3->MCR = 3; // on match, interrupt and stop
    LPC_TIM3->TCR = 0; // disable interrupt

    mbed::InterruptIn *zPin = this->ZeroPin.interrupt_pin();
    zPin->rise(Heater::ip_rose);
    NVIC_EnableIRQ(TIMER3_IRQn);    // Enable interrupt handler
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
    Heater::heat_enabled = enable_flag;
}

bool Heater::is_enabled() {
    return Heater::heat_enabled;
}


extern "C" void TIMER3_IRQHandler (void) {
    LPC_TIM3->IR |= 1 << 0;          // Clear MR0 interrupt flag
    Heater::getInstance()->heater_phase_start();
}
