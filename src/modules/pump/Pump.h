#ifndef PUMP_H
#define PUMP_H

#include <string>
using std::string;
#include <string.h>
#include <functional>
#include <stack>
#include <vector>

#include "libs/Module.h"
#include "nuts_bolts.h"

class Gcode;
class StepperMotor;


#define MAX_ROBOT_ACTUATORS 3


class Pump : public Module {
    public:
        Pump();
        void on_module_loaded();
        void on_gcode_received(void* argument);

        float get_default_acceleration() const { return default_acceleration; }
        float get_axis_position(int axis) const { return(this->machine_position[axis]); }
        void get_axis_position(float position[], size_t n= MAX_ROBOT_ACTUATORS) const { memcpy(position, this->machine_position, n*sizeof(float)); }
        int print_position(uint8_t subcode, char *buf, size_t bufsize) const;

        uint8_t register_motor(StepperMotor*);
        uint8_t get_number_registered_motors() const {return n_motors; }

        float get_flow_rate() const { return(this->get_flow_rate_n()); }
        float get_flow_rate_n() const { return(this->flow_rate_n); }
        float get_flow_rate_a() const { return(this->flow_rate_a); }
        void set_flow_rate();

        // gets accessed by Panel, Endstops, ZProbe
        std::vector<StepperMotor*> actuators;

    private:
        void load_config();
        void process_move(Gcode *gcode);
        void check_max_actuator_speeds();

        float machine_position[MAX_ROBOT_ACTUATORS]; // Last requested position, in millimeters, which is what we were requested to move to in the gcode after offsets applied but before compensation transform

        float seek_rate;                                     // Current rate for seeking moves ( mm/min )
        float feed_rate;                                     // Current rate for feeding moves ( mm/min )
        float seconds_per_minute;                            // for realtime speed change
        float default_acceleration;                          // the defualt accleration if not set for each axis

        float max_speeds[MAX_ROBOT_ACTUATORS];               // Setting : max allowable speed in mm/s for each axis

        uint8_t n_motors;                                    //count of the motors/axis registered

        float flow_rate_n;
        float flow_rate_a;
};


#endif
