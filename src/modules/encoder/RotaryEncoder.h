#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H

#include "Module.h"
#include "Pin.h"

#include "libs/Hook.h"

class RotaryEncoder : public Module {
    public:
        RotaryEncoder();
        static RotaryEncoder* instance;

        void on_module_loaded();
        uint32_t encoder_tick(uint32_t dummy);
        void on_idle(void* argument);
        void on_main_loop(void* argument);
        void reset_counter();

        int get_pos();

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
        Pin a_pin, b_pin;
        bool last_a;

        Hook *up_hook;
        Hook *down_hook;

        volatile struct {
            int pos;
        };
};


#endif  // ROTARY_ENCODER_H
