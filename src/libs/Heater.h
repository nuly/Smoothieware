#include "Pin.h"

class Heater {
    public:
        Heater();
        static void ip_rose();

        void set_delay_us(int microseconds);

        inline void heater_phase_wait() {
            HeaterPin.set(0);
        }
        inline void heater_phase_start() {
            if (heat_enabled) {
                HeaterPin.set(1);
            }
        }

        static void enable(bool enable_flag);
        static Heater *getInstance() { return instance; }

    private:
        static Heater *instance;
        static bool heat_enabled;

        Pin ZeroPin, HeaterPin;
};
