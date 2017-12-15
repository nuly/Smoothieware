#include <math.h>

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
#include "StreamOutputPool.h"

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

PotReader *PotReader::instance;
int PotReader::count;

PotReader::PotReader(mbed::I2C *i2c, char addr) {
    instance = this; // setup the Singleton instance of the heater

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

    override_ds = false;

    this->temp_pot[0].from_string(
        THEKERNEL->config->value(CHECKSUM("temp_pot0"))
        ->by_default("0.26")->as_string()
        );
    Adc::instance->enable_pin(&(this->temp_pot[0]));

    this->temp_pot[1].from_string(
        THEKERNEL->config->value(CHECKSUM("temp_pot1"))
        ->by_default("0.25")->as_string()
        );
    Adc::instance->enable_pin(&(this->temp_pot[1]));

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
    if (_pos > 100) {
        _pos = 100;
    } else if (_pos < 0) {
        _pos = 0;
    }

    return 90 + _pos * 0.1;
}

#define KK1 46.
#define KK2 8.15E-8
#define KK3 1.1E-3
#define KA  0.00105357142857139
#define KB  9.17815476190453

uint32_t
PotReader::pot_read_tick(uint32_t dummy) {
    float speed_pot_val = pot_val();
    int64_t speed = speed_pot_val * QVmax * 0.9;

    StepTicker::getInstance()->pump_speed(override_ds ? override_speed : speed);

    // forward is 0, backward is 1
    float heater_pot_val = get_pos();
//    heater_pot_val = HEATINVF(speed_pot_val * heater_pot_val);

    get_current_temp(true, 0);
    get_current_temp(true, 1);
    if (Heater::is_enabled()) {
        float dT = 0.25;
        float Tod = 0.1 * (heater_pot_val - get_current_temp(false)) / dT;
        float powerd = (Tod - (get_current_temp(false, 0) - get_current_temp(false, 1)) * (KK2 * speed + KK3)) /  KK1;
        if (powerd > 0) {
            heater = (KB - sqrt(200 * powerd))/KA;
        } else {
            heater = 8000;
        }

        // hardcore feedback law
        //        heater += 10 * (get_current_temp(false) - heater_pot_val);
        
        if (heater > 8000) {
            heater = 8000;
        }
        if (heater < 300) {
            heater = 300;
        }

        Heater::getInstance()->set_delay_us(override_ds ? override_delay : heater);
    }

    if (count++ % 5 == 0 && override_ds) {
        print_temp_line();
    }

    return 0;
}

float
PotReader::get_current_temp(bool read, int index) {
    static float last_good_val[] = {50, 50, 50};
    if (!read) {
        return last_good_val[index];
    }

    float beta = 3976.;
    float T0 = 25.;
    float maxr = 50000.;

    float res = 4700./
        (float(Adc::instance->get_max_value())/
         Adc::instance->read(&this->temp_pot[index]) - 1);

    if (res > 0) {
        // otherwise the logarithm would fail,
        // causing the board to crash
        last_good_val[index] = 1/(1/(273.15 + T0) + (logf(res/maxr)/beta)) - 273.15;
    }

    return last_good_val[index];
}


float
PotReader::get_current_temp(bool read) {
    return get_current_temp(read, 1);
}


void
PotReader::print_temp_line() {
    THEKERNEL->streams->printf("TEMPOUT %d %ld %ld %d %d %d\n", count, (int32_t)override_speed, override_delay, Adc::instance->read(&this->temp_pot[0]), Adc::instance->read(&this->temp_pot[1]), Adc::instance->get_max_value());
}

void
PotReader::set_override(int32_t delay, int64_t speed) {
    override_delay = delay;
    override_speed = speed;
    override_ds = speed > 0;
}
