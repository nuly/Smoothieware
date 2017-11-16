#include "RotaryEncoder.h"

#include "libs/Kernel.h"
#include "InterruptIn.h"

#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"

RotaryEncoder *RotaryEncoder::instance;
volatile int RotaryEncoder::pos;
volatile unsigned char RotaryEncoder::oldval;
volatile unsigned char RotaryEncoder::curval;

Timer RotaryEncoder::atimer;
Timer RotaryEncoder::btimer;

#define AFLAG 0x1
#define BFLAG 0x2

#define DEBOUNCE 50

RotaryEncoder::RotaryEncoder() {
    instance = this;

    this->a_pin.from_string(
//                    THEKERNEL->config->value(CHECKSUM("temp_re_a"))
//                    ->by_default("2.6")->as_string());
                    "0.25");
//                    "2.6");
    this->a_pin.as_input();
    this->a_pin.pull_up();

    this->b_pin.from_string(
//                    THEKERNEL->config->value(CHECKSUM("temp_re_b"))
//                    ->by_default("2.7")->as_string());
                    "0.26");
//                    "2.7");
    this->b_pin.as_input();
    this->b_pin.pull_up();

    this->minval = 
//        THEKERNEL->config->value(CHECKSUM("temp_re_min"))
//        ->by_default(0)->as_number();
        10;
    this->maxval = 
//        THEKERNEL->config->value(CHECKSUM("temp_re_max"))
//        ->by_default(50)->as_number();
        50;

    this->pos = (this->minval + this->maxval)/2;

    this->up_hook = NULL;
    this->down_hook = NULL;

    this->oldval = 0;
    this->curval = AFLAG | BFLAG;

    atimer.start();
    btimer.start();

    InterruptIn *a_interrupt = this->a_pin.interrupt_pin();
    InterruptIn *b_interrupt = this->b_pin.interrupt_pin();

    a_interrupt->rise(RotaryEncoder::encoder_ra);
    a_interrupt->fall(RotaryEncoder::encoder_fa);
    b_interrupt->rise(RotaryEncoder::encoder_rb);
    b_interrupt->fall(RotaryEncoder::encoder_fb);
}

void
RotaryEncoder::incpos() {
    pos++;
}

void
RotaryEncoder::decpos() {
    pos--;
}

void
RotaryEncoder::nop() {
}

void
RotaryEncoder::encoder_ra() {
    if (curval & AFLAG) return;
    if (atimer.read_us() < DEBOUNCE) return;
    oldval = curval;
    curval = curval | AFLAG;
    atimer.reset();
    update();
//    if (curval & BFLAG) pos++;
//    else pos--;
}


void
RotaryEncoder::encoder_fa() {
    if (!(curval & AFLAG)) return;
    if (atimer.read_us() < DEBOUNCE) return;
    oldval = curval;
    curval = curval & ~AFLAG;
    atimer.reset();
    update();
}

void
RotaryEncoder::encoder_rb() {
    if (curval & BFLAG) return;
    if (btimer.read_us() < DEBOUNCE) return;
    oldval = curval;
    curval = curval | BFLAG;
    btimer.reset();
    update();
}

void
RotaryEncoder::encoder_fb() {
    if (!(curval & BFLAG)) return;
    if (btimer.read_us() < DEBOUNCE) return;
    oldval = curval;
    curval = curval & ~BFLAG;
    btimer.reset();
    update();
}


void
RotaryEncoder::update() {
    switch ((curval << 2) | oldval) {
        case 0x0: case 0x5: case 0xA: case 0xF:
            break;
        case 0x1: case 0x7: case 0x8: case 0xE:
            pos--;
            break;
        case 0x2: case 0x4: case 0xB: case 0xD:
            pos++; break;

        // we missed something!
        case 0x6:
//            pos += 2;
            break;

        case 0x9:
//            pos -= 2;
            break;

        case 0x3:
        case 0xC:
            break;
    }
}


void
RotaryEncoder::reset_counter() {
    pos = 0;
}

int
RotaryEncoder::get_pos() {
    return 
//        (
//        (instance->b_pin.connected() ? 0x8 : 0x0) |
//        (instance->a_pin.connected() ? 0x4 : 0x0) |
//        (instance->b_pin.get() ? 0x2 : 0x0) |
//        (instance->a_pin.get() ? 0x1 : 0x0)) * 100 +
//     (curval * 10 + oldval) * 100 + 
     pos;
}

float
RotaryEncoder::normalized_pos() {
    return (pos - minval)/((float)(maxval - minval));
}
