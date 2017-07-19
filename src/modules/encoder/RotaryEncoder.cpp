#include "RotaryEncoder.h"

#include "libs/Kernel.h"
#include "Hook.h"
#include "SlowTicker.h"

RotaryEncoder* RotaryEncoder::instance= nullptr;

RotaryEncoder::RotaryEncoder() {
    instance = this;
    this->pos = 0;

    this->a_pin.from_string("1.29");
    this->b_pin.from_string("1.28");

    this->up_hook = NULL;
    this->down_hook = NULL;
}

void
RotaryEncoder::on_module_loaded() {
    THEKERNEL->slow_ticker->attach( 1000, this, &RotaryEncoder::encoder_tick );
    this->register_for_event(ON_IDLE);
    this->register_for_event(ON_MAIN_LOOP);

    this->last_a = this->a_pin.get();
}

uint32_t
RotaryEncoder::encoder_tick(uint32_t dummy) {
    bool aVal = this->a_pin.get();

    if (aVal != this->last_a) {
        // knob is rotating
        if (this->b_pin.get() != aVal) {
            // a changed first
            // clockwise
            this->pos ++;
            if ( this->up_hook != NULL ) {
                this->up_hook->call();
            }
        } else {
            // counter clockwise
            this->pos --;
            if ( this->down_hook != NULL ) {
                this->down_hook->call();
            }
        }
    }
    this->last_a = aVal;

    return 0;
}

void
RotaryEncoder::on_main_loop(void *argument) {
    this->encoder_tick(0);
}

void
RotaryEncoder::on_idle(void *argument) {
    this->encoder_tick(0);
}

void
RotaryEncoder::reset_counter() {
    pos = 0;
}

int
RotaryEncoder::get_pos() {
    return pos;
}

