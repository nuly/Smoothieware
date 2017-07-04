#include "libs/Module.h"
#include "libs/Kernel.h"

#include "Pump.h"
#include "Pin.h"
#include "StepperMotor.h"
#include "Gcode.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "StepTicker.h"
#include "checksumm.h"
#include "utils.h"
#include "Config.h"
#include "ConfigValue.h"
#include "libs/StreamOutput.h"
#include "StreamOutputPool.h"
#include "GcodeDispatch.h"

#include "mbed.h" // for us_ticker_read()
#include "mri.h"

#include <fastmath.h>
#include <string>
#include <algorithm>

#define  default_seek_rate_checksum          CHECKSUM("default_seek_rate")
#define  default_feed_rate_checksum          CHECKSUM("default_feed_rate")
#define  x_axis_max_speed_checksum           CHECKSUM("x_axis_max_speed")
#define  y_axis_max_speed_checksum           CHECKSUM("y_axis_max_speed")
#define  z_axis_max_speed_checksum           CHECKSUM("z_axis_max_speed")
#define  segment_z_moves_checksum            CHECKSUM("segment_z_moves")

#define  step_pin_checksum                   CHECKSUM("step_pin")
#define  dir_pin_checksum                    CHEKCSUM("dir_pin")
#define  en_pin_checksum                     CHECKSUM("en_pin")

#define  steps_per_mm_checksum               CHECKSUM("steps_per_mm")
#define  max_rate_checksum                   CHECKSUM("max_rate")
#define  acceleration_checksum               CHECKSUM("acceleration")
#define  z_acceleration_checksum             CHECKSUM("z_acceleration")

#define  alpha_checksum                      CHECKSUM("alpha")
#define  beta_checksum                       CHECKSUM("beta")
#define  gamma_checksum                      CHECKSUM("gamma")

#define PI 3.14159265358979323846F // force to be float, do not use M_PI

Pump::Pump()
{
    memset(this->machine_position, 0, sizeof machine_position);
    this->n_motors= 0;
    this->flow_rate_n= 0.;
    this->flow_rate_a= 0.;
}

//Called when the module has just been loaded
void Pump::on_module_loaded()
{
    this->register_for_event(ON_GCODE_RECEIVED);

    // Configuration
    this->load_config();
}

#define ACTUATOR_CHECKSUMS(X) {     \
    CHECKSUM(X "_step_pin"),        \
    CHECKSUM(X "_dir_pin"),         \
    CHECKSUM(X "_en_pin"),          \
    CHECKSUM(X "_steps_per_mm"),    \
    CHECKSUM(X "_max_rate"),        \
    CHECKSUM(X "_acceleration")     \
}

void Pump::load_config()
{
    this->feed_rate           = THEKERNEL->config->value(default_feed_rate_checksum   )->by_default(  100.0F)->as_number();
    this->seek_rate           = THEKERNEL->config->value(default_seek_rate_checksum   )->by_default(  100.0F)->as_number();

    // in mm/sec but specified in config as mm/min
    this->max_speeds[X_AXIS]  = THEKERNEL->config->value(x_axis_max_speed_checksum    )->by_default(60000.0F)->as_number() / 60.0F;
    this->max_speeds[Y_AXIS]  = THEKERNEL->config->value(y_axis_max_speed_checksum    )->by_default(60000.0F)->as_number() / 60.0F;
    this->max_speeds[Z_AXIS]  = THEKERNEL->config->value(z_axis_max_speed_checksum    )->by_default(  300.0F)->as_number() / 60.0F;

     // Make our Primary XYZ StepperMotors, and potentially A B C
    uint16_t const checksums[][6] = {
        ACTUATOR_CHECKSUMS("alpha"), // X
        ACTUATOR_CHECKSUMS("beta"),  // Y
        ACTUATOR_CHECKSUMS("gamma"), // Z
        #if MAX_ROBOT_ACTUATORS > 3
        ACTUATOR_CHECKSUMS("delta"),   // A
        #if MAX_ROBOT_ACTUATORS > 4
        ACTUATOR_CHECKSUMS("epsilon"), // B
        #if MAX_ROBOT_ACTUATORS > 5
        ACTUATOR_CHECKSUMS("zeta")     // C
        #endif
        #endif
        #endif
    };

    // default acceleration setting, can be overriden with newer per axis settings
//    this->default_acceleration= THEKERNEL->config->value(acceleration_checksum)->by_default(100.0F )->as_number(); // Acceleration is in mm/s^2

    // make each motor
    for (size_t a = 0; a < MAX_ROBOT_ACTUATORS; a++) {
        Pin pins[3]; //step, dir, enable
        for (size_t i = 0; i < 3; i++) {
            pins[i].from_string(THEKERNEL->config->value(checksums[a][i])->by_default("nc")->as_string())->as_output();
        }

        if(!pins[0].connected() || !pins[1].connected()) { // step and dir must be defined, but enable is optional
            if(a <= Y_AXIS) {
                THEKERNEL->streams->printf("FATAL: motor %c is not defined in config\n", 'X'+a);
                n_motors= a; // we only have this number of motors
                return;
            }
            break; // if any pin is not defined then the axis is not defined (and axis need to be defined in contiguous order)
        }

        StepperMotor *sm = new StepperMotor(pins[0], pins[1], pins[2]);
        // register this motor (NB This must be 0,1,2) of the actuators array
        uint8_t n= register_motor(sm);
        if(n != a) {
            // this is a fatal error
            THEKERNEL->streams->printf("FATAL: motor %d does not match index %d\n", n, a);
            return;
        }

//        actuators[a]->change_steps_per_mm(THEKERNEL->config->value(checksums[a][3])->by_default(a == 2 ? 2560.0F : 80.0F)->as_number());
//        actuators[a]->set_max_rate(THEKERNEL->config->value(checksums[a][4])->by_default(30000.0F)->as_number()/60.0F); // it is in mm/min and converted to mm/sec
//        actuators[a]->set_acceleration(THEKERNEL->config->value(checksums[a][5])->by_default(NAN)->as_number()); // mm/secs²
    }

    check_max_actuator_speeds(); // check the configs are sane
}

uint8_t Pump::register_motor(StepperMotor *motor)
{
    // register this motor with the step ticker
    THEKERNEL->step_ticker->register_motor(motor);
    if(n_motors >= k_max_actuators) {
        // this is a fatal error
        THEKERNEL->streams->printf("FATAL: too many motors, increase k_max_actuators\n");
        __debugbreak();
    }
    actuators.push_back(motor);
    motor->set_motor_id(n_motors);
    return n_motors++;
}

int Pump::print_position(uint8_t subcode, char *buf, size_t bufsize) const
{
    // get real time current actuator position in mm
    int n = 0;

    for (int i = 0; i < n_motors; ++i) {
        // current actuator position
//        n += snprintf(&buf[n], bufsize-n, " %d:%1.4f", i, actuators[i]->get_current_position());
    }

    return n;
}

// this does a sanity check that actuator speeds do not exceed steps rate capability
// we will override the actuator max_rate if the combination of max_rate and steps/sec exceeds base_stepping_frequency
void Pump::check_max_actuator_speeds()
{
        /*
    for (size_t i = 0; i < n_motors; i++) {
        float step_freq = actuators[i]->get_max_rate() * actuators[i]->get_steps_per_mm();
        if (step_freq > THEKERNEL->base_stepping_frequency) {
            actuators[i]->set_max_rate(floorf(THEKERNEL->base_stepping_frequency / actuators[i]->get_steps_per_mm()));
            THEKERNEL->streams->printf("WARNING: actuator %d rate exceeds base_stepping_frequency * ..._steps_per_mm: %f, setting to %f\n", i, step_freq, actuators[i]->get_max_rate());
        }
    } */
}

void Pump::on_gcode_received(void *argument)
{
}
