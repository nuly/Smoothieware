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

    private:
        Pin speed_pot;

        float TOP_R;
        float MAX_R;

        int minval, maxval;

        char _addr;
        signed char _pos = 25;
        mbed::I2C *_i2c;
};

#endif  // POT_READER_H
