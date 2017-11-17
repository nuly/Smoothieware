#include "PotReader.h"

#include "libs/Kernel.h"

#include "StepTicker.h"
#include "SlowTicker.h"
#include "Heater.h"

#include "libs/Adc.h"
#include "libs/constants.h"

#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"

#define HEATINVLEN 20

float HEATINV[] = 
{
1.0, 0.79804, 0.74109, 0.69896, 0.66368, 0.63237, 0.60358, 0.57645, 0.55042, 0.52505, 0.5, 0.47495, 0.44958, 0.42355, 0.39642, 0.36763, 0.33632, 0.30104, 0.25891, 0.20196, 0.
};

float HEATINVF(float powfact) {
    int idx = powfact * HEATINVLEN;
    float di = powfact * HEATINVLEN - idx;
    return HEATINV[idx] * (1 - di) + HEATINV[idx+1] * di;
}

PotReader::PotReader(mbed::I2C *i2c, char addr) {
    _i2c = i2c;
    _addr = addr;

    this->TOP_R = THEKERNEL->config->value(CHECKSUM("TOP_R"))
        ->by_default(4.7F)->as_number();
    this->MAX_R = THEKERNEL->config->value(CHECKSUM("MAX_R"))
        ->by_default(10.F)->as_number();

    this->speed_pot.from_string(
        THEKERNEL->config->value(CHECKSUM("speed_pot"))
        ->by_default("0.24")->as_string()
        );
    Adc::instance->enable_pin(&(this->speed_pot));
    THEKERNEL->slow_ticker->attach(20, this, &PotReader::pot_read_tick);
}


float
PotReader::pot_val() {
    float val = 
        1 - Adc::instance->read(&this->speed_pot)/((float)Adc::instance->get_max_value());

    if (val < 0.) {
        val = 0.;
    } else if (val >= MAX_R/(TOP_R + MAX_R)) {
        val = MAX_R/(TOP_R + MAX_R);
    }
    val = 1 - val; // [4.7/14.7, 1]
    val = (TOP_R/val - TOP_R)/MAX_R;
    return val;
}

float
PotReader::get_pos() {
    char buf;

    _i2c->read(_addr << 1, &buf, 1);
    _pos += (signed char)buf;
    if (_pos > 50) {
        _pos = 50;
    } else if (_pos < 0) {
        _pos = 0;
    }

    return _pos * 0.02;
}

uint32_t
PotReader::pot_read_tick(uint32_t dummy) {
    float speed_pot_val = pot_val();
    int64_t speed = speed_pot_val * QVmax;
    StepTicker::getInstance()->pump_speed(speed);

    // forward is 0, backward is 1
    float heater_pot_val = get_pos();
    heater_pot_val = HEATINVF(speed_pot_val * heater_pot_val);
    int32_t heater = 8333 * heater_pot_val;

    Heater::getInstance()->set_delay_us(heater);

    return 0;
}
