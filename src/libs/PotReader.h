#ifndef POT_READER_H
#define POT_READER_H

#include "mbed.h"
#include "Pin.h"

class PotReader {
    public:
        PotReader(mbed::I2C *i2c, char addr);
        float pot_val();
        float get_pos();
        uint32_t pot_read_tick(uint32_t dummy);
        void set_override(int32_t, int64_t);

        static PotReader *getInstance() { return instance; }

        void print_temp_line();

        float get_current_temp(bool read, int index);
        float get_current_temp(bool read);

    private:
        static PotReader *instance;
        Pin speed_pot;
        Pin temp_pot[2];

        float TOP_R;
        float MAX_R;

        int minval, maxval;

        char _addr;
        signed char _pos = 25;
        mbed::I2C *_i2c;

        bool override_ds;
        int32_t override_delay = 0;
        int64_t override_speed = 0;

        int32_t heater = 8000;

        static int count;
};

#endif  // POT_READER_H
