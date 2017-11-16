#ifndef POT_READER_H
#define POT_READER_H

#include "Pin.h"
#include "RotaryEncoder.h"

class PotReader {
    public:
        PotReader(RotaryEncoder *TRE);
        float pot_val();
        uint32_t pot_read_tick(uint32_t dummy);

    private:
        Pin speed_pot;

        float TOP_R;
        float MAX_R;

        int minval, maxval;

        RotaryEncoder *TRE;
};

#endif  // POT_READER_H
