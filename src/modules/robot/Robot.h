/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ROBOT_H
#define ROBOT_H

#include <string>
using std::string;
#include <string.h>
#include <functional>
#include <stack>
#include <vector>

#include "libs/Module.h"
#include "ActuatorCoordinates.h"
#include "nuts_bolts.h"

class Gcode;
class BaseSolution;
class StepperMotor;

// 9 WCS offsets
#define MAX_WCS 9UL

class Robot : public Module {
    public:
        Robot();
        void on_module_loaded();
        void on_gcode_received(void* argument);

        void reset_axis_position(float position, int axis);
        void reset_axis_position(float x, float y, float z);
        void reset_actuator_position(const ActuatorCoordinates &ac);
        void reset_position_from_current_actuator_position();
        float get_seconds_per_minute() const { return seconds_per_minute; }
        float get_z_maxfeedrate() const { return this->max_speeds[Z_AXIS]; }
        float get_default_acceleration() const { return default_acceleration; }
        float get_feed_rate() const;
        float get_s_value() const { return s_value; }
        void set_s_value(float s) { s_value= s; }
        void check_max_actuator_speeds();
        float get_axis_position(int axis) const { return(this->machine_position[axis]); }
        void get_axis_position(float position[], size_t n= 3) const { memcpy(position, this->machine_position, n*sizeof(float)); }
        void get_current_machine_position(float *pos) const;
        void print_position(uint8_t subcode, std::string& buf) const;
        bool delta_move(const float delta[], float rate_mm_s, uint8_t naxis);
        uint8_t register_motor(StepperMotor*);
        uint8_t get_number_registered_motors() const {return n_motors; }

        BaseSolution* arm_solution;                           // Selected Arm solution ( millimeters to step calculation )

        // gets accessed by Panel, Endstops, ZProbe
        std::vector<StepperMotor*> actuators;

        struct {
            bool absolute_mode:1;                             // true for absolute mode ( default ), false for relative mode
            bool disable_arm_solution:1;                      // set to disable the arm solution
            bool segment_z_moves:1;
            bool is_g123:1;
            bool soft_endstop_enabled:1;
            bool soft_endstop_halt:1;
        };

    private:
        enum MOTION_MODE_T {
            NONE,
            SEEK, // G0
            LINEAR, // G1
//            CW_ARC, // G2
//            CCW_ARC // G3
        };

        void load_config();
        bool append_milestone(const float target[], float rate_mm_s);
        bool append_line( Gcode* gcode, const float target[], float rate_mm_s);
        void process_move(Gcode *gcode, enum MOTION_MODE_T);
        bool is_homed(uint8_t i) const;

        float theta(float x, float y);

        float machine_position[k_max_actuators]; // Last requested position, in millimeters, which is what we were requested to move to in the gcode after offsets applied but before compensation transform
        float compensated_machine_position[k_max_actuators]; // Last machine position, which is the position before converting to actuator coordinates (includes compensation transform)

        float seek_rate;                                     // Current rate for seeking moves ( mm/min )
        float feed_rate;                                     // Current rate for feeding moves ( mm/min )
        float mm_per_line_segment;                           // Setting : Used to split lines into segments
        float delta_segments_per_second;                     // Setting : Used to split lines into segments for delta based on speed
        float seconds_per_minute;                            // for realtime speed change
        float default_acceleration;                          // the defualt accleration if not set for each axis
        float s_value;                                       // modal S value

        // Number of arc generation iterations by small angle approximation before exact arc trajectory
        // correction. This parameter may be decreased if there are issues with the accuracy of the arc
        // generations. In general, the default value is more than enough for the intended CNC applications
        // of grbl, and should be on the order or greater than the size of the buffer to help with the
        // computational efficiency of generating arcs.
        float max_speeds[3];                                 // Setting : max allowable speed in mm/s for each axis
        float max_speed;                                     // Setting : maximum feedrate in mm/s as specified by F parameter

        float soft_endstop_min[3], soft_endstop_max[3];

        uint8_t n_motors;                                    //count of the motors/axis registered

        // Used by Planner
        friend class Planner;
};


#endif
