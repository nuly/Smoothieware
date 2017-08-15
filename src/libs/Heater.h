#include "Pin.h"

class Heater {
    public:
        Heater();
        static void ip_rose();

        void set_delay_us(int microseconds);

        inline void heater_disable() {
            HeaterPin.set(0);
        }
        inline void heater_enable() {
            HeaterPin.set(1);
        }

        static Heater *getInstance() { return instance; }

    private:
        static Heater *instance;

        Pin ZeroPin, HeaterPin;
};
