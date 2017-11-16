#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H

#include "Pin.h"

#include "mbed.h"
#include "libs/Hook.h"

class RotaryEncoder {
    public:
        RotaryEncoder();

        static void encoder_ra();
        static void encoder_rb();
        static void encoder_fa();
        static void encoder_fb();
        static void incpos();
        static void decpos();
        static void nop();

        static void update();

        void reset_counter();

        static int get_pos();
        float normalized_pos();

        template<typename T> RotaryEncoder *up_attach( T *optr, uint32_t ( T::*fptr )( uint32_t ) )
        {
            this->up_hook = new Hook();
            this->up_hook->attach(optr, fptr);
            return this;
        }

        template<typename T> RotaryEncoder *down_attach( T *optr, uint32_t ( T::*fptr )( uint32_t ) )
        {
            this->down_hook = new Hook();
            this->down_hook->attach(optr, fptr);
            return this;
        }

    private:
        static RotaryEncoder *instance;

        Pin a_pin, b_pin;

        Hook *up_hook;
        Hook *down_hook;

        int maxval, minval;

        static volatile int pos;
        static volatile unsigned char oldval;
        static volatile unsigned char curval;

        static Timer atimer;
        static Timer btimer;
};


#endif  // ROTARY_ENCODER_H
