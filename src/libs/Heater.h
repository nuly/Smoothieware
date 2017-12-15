#include "Pin.h"
#include "gpio.h"

class Heater {
    public:
        Heater(GPIO*);
        static void ip_rose();

        void set_delay_us(int microseconds);

        inline void heater_phase_wait() {
            if (this->led_to_blink) {
                this->led_to_blink->write(0);
            }
            this->HeaterPin.set(0);
        }
        inline void heater_phase_start() {
            if (this->led_to_blink) {
                this->count += 1;
                if (this->count > 60) {
                    this->led_to_blink->write(1);
                }
                if (this->count >= 120) {
                    this->count = 0;
                }
            }
            if (this->heat_enabled) {
                this->HeaterPin.set(1);
            }
        }

        static void enable(bool enable_flag);
        static bool is_enabled();
        static Heater *getInstance() { return instance; }

    private:
        static Heater *instance;
        static bool heat_enabled;

        static GPIO *led_to_blink;

        static int count;

        Pin ZeroPin, HeaterPin;
};
