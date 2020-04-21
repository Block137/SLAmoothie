/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl) with additions from Sungeun K. Jeon (https://github.com/chamnit/grbl)
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"

#include "Robot.h"
#include "Planner.h"
#include "Conveyor.h"
#include "Pin.h"
#include "StepperMotor.h"
#include "Gcode.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "arm_solutions/BaseSolution.h"
#include "arm_solutions/CartesianSolution.h"
#include "arm_solutions/GalvoSolution.h"
#include "StepTicker.h"
#include "checksumm.h"
#include "utils.h"
#include "ConfigValue.h"
#include "libs/StreamOutput.h"
#include "StreamOutputPool.h"
//#include "ExtruderPublicAccess.h"
#include "GcodeDispatch.h"
#include "ActuatorCoordinates.h"
#include "EndstopsPublicAccess.h"

#include "mbed.h" // for us_ticker_read()
#include "mri.h"

#include <fastmath.h>
#include <string>
#include <algorithm>

#define  default_seek_rate_checksum          CHECKSUM("default_seek_rate")
#define  default_feed_rate_checksum          CHECKSUM("default_feed_rate")
#define  mm_per_line_segment_checksum        CHECKSUM("mm_per_line_segment")
#define  delta_segments_per_second_checksum  CHECKSUM("delta_segments_per_second")
#define  x_axis_max_speed_checksum           CHECKSUM("x_axis_max_speed")
#define  y_axis_max_speed_checksum           CHECKSUM("y_axis_max_speed")
#define  z_axis_max_speed_checksum           CHECKSUM("z_axis_max_speed")
#define  segment_z_moves_checksum            CHECKSUM("segment_z_moves")

// arm solutions
#define  arm_solution_checksum               CHECKSUM("arm_solution")
#define  cartesian_checksum                  CHECKSUM("cartesian")
#define  galvo_checksum                      CHECKSUM("galvo")

// new-style actuator stuff
#define  actuator_checksum                   CHEKCSUM("actuator")

#define  step_pin_checksum                   CHECKSUM("step_pin")
#define  dir_pin_checksum                    CHEKCSUM("dir_pin")
#define  en_pin_checksum                     CHECKSUM("en_pin")

#define  max_speed_checksum                  CHECKSUM("max_speed")
#define  acceleration_checksum               CHECKSUM("acceleration")
#define  z_acceleration_checksum             CHECKSUM("z_acceleration")

#define  alpha_checksum                      CHECKSUM("alpha")
#define  beta_checksum                       CHECKSUM("beta")
#define  gamma_checksum                      CHECKSUM("gamma")

#define laser_module_default_power_checksum     CHECKSUM("laser_module_default_power")

#define enable_checksum                    CHECKSUM("enable")
#define halt_checksum                      CHECKSUM("halt")
#define soft_endstop_checksum              CHECKSUM("soft_endstop")
#define xmin_checksum                      CHECKSUM("x_min")
#define ymin_checksum                      CHECKSUM("y_min")
#define zmin_checksum                      CHECKSUM("z_min")
#define xmax_checksum                      CHECKSUM("x_max")
#define ymax_checksum                      CHECKSUM("y_max")
#define zmax_checksum                      CHECKSUM("z_max")

#define PI        3.14159265358979323846F // force to be float, do not use M_PI
#define DEG2RAD   0.01745329251994329576923690768489F

//#define DEBUG_PRINTF THEKERNEL->streams->printf
#define DEBUG_PRINTF(...)

// The Robot converts GCodes into actual movements, and then adds them to the Planner, which passes them to the Conveyor so they can be added to the queue

Robot::Robot()
{
    this->absolute_mode = true;
    memset(this->machine_position, 0, sizeof machine_position);
    this->arm_solution = NULL;
    seconds_per_minute = 60.0F;
    this->disable_arm_solution= false;
    this->n_motors= 0;
}

//Called when the module has just been loaded
void Robot::on_module_loaded()
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

void Robot::load_config()
{
    // Arm solutions are used to convert positions in millimeters into position in steps for each stepper motor.
    // While for a cartesian arm solution, this is a simple multiplication, in other, less simple cases, there is some serious math to be done.
    // To make adding those solution easier, they have their own, separate object.
    // Here we read the config to find out which arm solution to use
//    if (this->arm_solution) delete this->arm_solution;
//    int solution_checksum = get_checksum(THEKERNEL->config->value(arm_solution_checksum)->by_default("cartesian")->as_string());
    // Note checksums are not const expressions when in debug mode, so don't use switch
//    if(solution_checksum == galvo_checksum) {
    this->arm_solution = new GalvoSolution(THEKERNEL->config);
//    } else {
//        this->arm_solution = new CartesianSolution(THEKERNEL->config);
//    }

    this->feed_rate           = THEKERNEL->config->value(default_feed_rate_checksum   )->by_default(  600.0F)->as_number();
    this->seek_rate           = THEKERNEL->config->value(default_seek_rate_checksum   )->by_default( 1000.0F)->as_number();
    this->mm_per_line_segment = THEKERNEL->config->value(mm_per_line_segment_checksum )->by_default(    5.0F)->as_number();
    this->delta_segments_per_second = THEKERNEL->config->value(delta_segments_per_second_checksum )->by_default(0.0F)->as_number();

    // in mm/sec but specified in config as mm/min
    this->max_speeds[X_AXIS]  = THEKERNEL->config->value(x_axis_max_speed_checksum    )->by_default(100000.0F)->as_number() / 60.0F;
    this->max_speeds[Y_AXIS]  = THEKERNEL->config->value(y_axis_max_speed_checksum    )->by_default(100000.0F)->as_number() / 60.0F;
    this->max_speeds[Z_AXIS]  = THEKERNEL->config->value(z_axis_max_speed_checksum    )->by_default(  2000.0F)->as_number() / 60.0F;
    this->max_speed           = THEKERNEL->config->value(max_speed_checksum           )->by_default(140000.0F)->as_number() / 60.0F;

    this->segment_z_moves     = THEKERNEL->config->value(segment_z_moves_checksum     )->by_default(true)->as_bool();

    // default s value for laser
    this->s_value             = THEKERNEL->config->value(laser_module_default_power_checksum)->by_default(0.8F)->as_number();

     // Make our Primary XYZ StepperMotors, and potentially A B C
    uint16_t const motor_checksums[][6] = {
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
    this->default_acceleration= THEKERNEL->config->value(acceleration_checksum)->by_default(100.0F )->as_number(); // Acceleration is in mm/s^2
    float dac_step_size = (float)THEKERNEL->step_ticker->get_dac_step_size();

    // make each motor
    for (size_t a = 0; a < MAX_ROBOT_ACTUATORS; a++) { //alpha & beta are not steppers
        Pin pins[3]; //step, dir, enable
        for (size_t i = 0; i < 3; i++) {
            pins[i].from_string(THEKERNEL->config->value(motor_checksums[a][i])->by_default("nc")->as_string())->as_output();
        }

        if((!pins[0].connected() || !pins[1].connected()) && a > Y_AXIS) { // step and dir must be defined, but enable is optional, Y share pins with X
            if(a <= Z_AXIS) {
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

        if(a <= 1) {
            actuators[a]->change_steps_per_mm((THEKERNEL->config->value(motor_checksums[a][3])->by_default(2184.5F)->as_number() / dac_step_size) / DEG2RAD );
        } else {
            actuators[a]->change_steps_per_mm(THEKERNEL->config->value(motor_checksums[a][3])->by_default(100.0F)->as_number());
        }
        actuators[a]->set_max_rate((THEKERNEL->config->value(motor_checksums[a][4])->by_default(36000.0F)->as_number() / 60.0F ) * DEG2RAD); // it is in mm/min and converted to mm/sec
        actuators[a]->set_acceleration(THEKERNEL->config->value(motor_checksums[a][5])->by_default(NAN)->as_number()); // mm/secs²
    }

    check_max_actuator_speeds(); // check the configs are sane

    // if we have not specified a z acceleration see if the legacy config was set
    if(isnan(actuators[Z_AXIS]->get_acceleration())) {
        float acc= THEKERNEL->config->value(z_acceleration_checksum)->by_default(NAN)->as_number(); // disabled by default
        if(!isnan(acc)) {
            actuators[Z_AXIS]->set_acceleration(acc);
        }
    }

    // initialise actuator positions to current cartesian position (X0 Y0 Z0)
    // so the first move can be correct if homing is not performed
//    #if MAX_ROBOT_ACTUATORS > 3
    // initialize any extra axis to machine position
    for (size_t i = Z_AXIS; i < n_motors; i++) {
         actuators[i]->change_last_milestone(machine_position[i]);
    }
//    #endif

    soft_endstop_enabled= THEKERNEL->config->value(soft_endstop_checksum, enable_checksum)->by_default(false)->as_bool();
    soft_endstop_halt= THEKERNEL->config->value(soft_endstop_checksum, halt_checksum)->by_default(true)->as_bool();

    soft_endstop_min[X_AXIS]= THEKERNEL->config->value(soft_endstop_checksum, xmin_checksum)->by_default(NAN)->as_number();
    soft_endstop_min[Y_AXIS]= THEKERNEL->config->value(soft_endstop_checksum, ymin_checksum)->by_default(NAN)->as_number();
    soft_endstop_min[Z_AXIS]= THEKERNEL->config->value(soft_endstop_checksum, zmin_checksum)->by_default(NAN)->as_number();
    soft_endstop_max[X_AXIS]= THEKERNEL->config->value(soft_endstop_checksum, xmax_checksum)->by_default(NAN)->as_number();
    soft_endstop_max[Y_AXIS]= THEKERNEL->config->value(soft_endstop_checksum, ymax_checksum)->by_default(NAN)->as_number();
    soft_endstop_max[Z_AXIS]= THEKERNEL->config->value(soft_endstop_checksum, zmax_checksum)->by_default(NAN)->as_number();
}

uint8_t Robot::register_motor(StepperMotor *motor)
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

// this does a sanity check that actuator speeds do not exceed steps rate capability
// we will override the actuator max_rate if the combination of max_rate and steps/sec exceeds base_stepping_frequency
void Robot::check_max_actuator_speeds()
{
    for (size_t i = 0; i < n_motors; i++) {
        float step_freq = actuators[i]->get_max_rate() * actuators[i]->get_steps_per_mm();
        if (step_freq > (float)THEKERNEL->base_stepping_frequency) {
            actuators[i]->set_max_rate(floorf(THEKERNEL->base_stepping_frequency / actuators[i]->get_steps_per_mm()));
            THEKERNEL->streams->printf("WARNING: actuator %d rate exceeds base_stepping_frequency * ..._steps_per_mm: %f, setting to %f\n", i, step_freq, actuators[i]->get_max_rate());
        }
    }

    float mm_per_rad;
    float cartesian[] = {1.0F, 0.0F};
    ActuatorCoordinates actuator_rad;

    arm_solution->cartesian_to_actuator( cartesian, actuator_rad );
    if(actuator_rad[0] == 0.0F) return;
    mm_per_rad = cartesian[0] / actuator_rad[0];

    float safe_speed = actuators[X_AXIS]->get_max_rate() * mm_per_rad * 0.9F;

    if(this->max_speed > safe_speed) {
        this->max_speed = safe_speed;
    }
}

void Robot::get_current_machine_position(float *pos) const
{
    // get real time current actuator position in mm
    ActuatorCoordinates current_position{
        actuators[X_AXIS]->get_current_position(),
        actuators[Y_AXIS]->get_current_position(),
        actuators[Z_AXIS]->get_current_position()
    };

    // get machine position from the actuator position using FK
    arm_solution->actuator_to_cartesian(current_position, pos);
}

void Robot::print_position(uint8_t subcode, std::string& res) const
{
    // M114 just does it the old way uses machine_position and does inverse transforms to get the requested position
    uint32_t n = 0;
    char buf[64];
    n = snprintf(buf, sizeof(buf), "MP: X:%1.4f Y:%1.4f Z:%1.4f A:%1.4f", machine_position[X_AXIS], machine_position[Y_AXIS], machine_position[Z_AXIS], machine_position[A_AXIS]);

    if(n > sizeof(buf)) n= sizeof(buf);
    res.append(buf, n);
}

//A GCode has been received
//See if the current Gcode line has some orders for us
void Robot::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    enum MOTION_MODE_T motion_mode= NONE;

    if( gcode->has_g) {
        switch( gcode->g ) {
            case 0:  motion_mode = SEEK;    break;
            case 1:  motion_mode = LINEAR;  break;
            case 4: { // G4 Dwell
                uint32_t delay_ms = 0;
                if (gcode->has_letter('P')) {
                    if(THEKERNEL->is_grbl_mode()) {
                        // in grbl mode (and linuxcnc) P is decimal seconds
                        float f= gcode->get_value('P');
                        delay_ms= f * 1000.0F;

                    }else{
                        // in reprap P is milliseconds, they always have to be different!
                        delay_ms = gcode->get_int('P');
                    }
                }
                if (gcode->has_letter('S')) {
                    delay_ms += gcode->get_int('S') * 1000;
                }
                if (delay_ms > 0) {
                    // drain queue
                    THEKERNEL->conveyor->wait_for_idle();
                    // wait for specified time
                    uint32_t start = us_ticker_read(); // mbed call
                    while ((us_ticker_read() - start) < delay_ms * 1000) {
                        THEKERNEL->call_event(ON_IDLE, this);
                        if(THEKERNEL->is_halted()) return;
                    }
                }
            }
            break;

            case 90: this->absolute_mode = true; break;
            case 91: this->absolute_mode = false; break;

            case 92: {
                // making current position whatever the coordinate arguments are
                #if MAX_ROBOT_ACTUATORS > 3
                if(gcode->subcode == 0 && gcode->get_num_args() > 0) {
                    if(gcode->has_letter('Z')){
                        float ap= gcode->get_value('Z');
                        machine_position[Z_AXIS]= ap;
                        actuators[Z_AXIS]->change_last_milestone(ap); // this updates the last_milestone in the actuator
                    }
                    for (int i = A_AXIS; i < n_motors; i++) {
                        // ABC just need to set machine_position if specified
                        char axis= 'A'+i-3;
                        float ap= gcode->get_value(axis);
                        if((ap == 0) && gcode->has_letter(axis)) {
                            machine_position[i]= ap;
                            actuators[i]->change_last_milestone(ap); // this updates the last_milestone in the actuator
                        }
                    }
                }
                #endif

                return;
            }
        }
    } 
    else {
        if( gcode->has_m) {
            switch( gcode->m ) {
                // case 0: // M0 feed hold, (M0.1 is release feed hold, except we are in feed hold)
                //     if(THEKERNEL->is_grbl_mode()) THEKERNEL->set_feed_hold(gcode->subcode == 0);
                //     break;
                case 17:
                    if(gcode->subcode == 1) {
                        THEKERNEL->step_ticker->configDAC8760();
                    }
                    else {
                        THEKERNEL->call_event(ON_ENABLE, (void*)1); // turn all enable pins on
                    }
                    break;

                case 18: // this allows individual motors to be turned off, no parameters falls through to turn all off
                    if(gcode->get_num_args() > 0) {
                        // bitmap of motors to turn off, where bit 1:X, 2:Y, 3:Z, 4:A, 5:B, 6:C
                        uint32_t bm= 0;
                        for (int i = 0; i < n_motors; ++i) {
                            char axis= (i <= Z_AXIS ? 'X'+i : 'A'+(i-3));
                            if(gcode->has_letter(axis)) bm |= (0x02<<i); // set appropriate bit
                        }

                        THEKERNEL->conveyor->wait_for_idle();
                        THEKERNEL->call_event(ON_ENABLE, (void *)bm);
                        break;
                    }
                    // fall through
                case 84:
                    THEKERNEL->conveyor->wait_for_idle();
                    THEKERNEL->call_event(ON_ENABLE, nullptr); // turn all enable pins off
                    break;

                case 92: // M92 - set steps per mm
                    for (int i = 0; i < n_motors; ++i) {
                        char axis= (i <= Z_AXIS ? 'X'+i : 'A'+(i-A_AXIS));
                        gcode->stream->printf("%c:%f ", axis, actuators[i]->get_steps_per_mm());
                    }
                    gcode->add_nl = true;
                    return;

                case 114:{
                    std::string buf;
                    print_position(gcode->subcode, buf);
                    gcode->txt_after_ok.append(buf);
                    return;
                }

                case 203: // M203 Set maximum feedrates in mm/sec, M203.1 set maximum actuator feedrates
                        if(gcode->get_num_args() == 0) {
                            for (size_t i = X_AXIS; i <= Z_AXIS; i++) {
                                gcode->stream->printf(" %c: %g ", 'X' + i, gcode->subcode == 0 ? this->max_speeds[i] : actuators[i]->get_max_rate());
                            }
                            if(gcode->subcode == 1) {
                                for (size_t i = A_AXIS; i < n_motors; i++) {
                                    gcode->stream->printf(" %c: %g ", 'A' + i - A_AXIS, actuators[i]->get_max_rate());
                                }
                            }else{
                                gcode->stream->printf(" S: %g ", this->max_speed);
                            }

                            gcode->add_nl = true;

                        }else{
                            if(gcode->has_letter('S')) max_speed = gcode->get_value('S');
                            if(gcode->has_letter('Z')) actuators[Z_AXIS]->set_max_rate( gcode->get_value('Z') );
                            for (size_t i = X_AXIS; i <= Y_AXIS; i++) {
                                if (gcode->has_letter('X' + i)) {
                                    float v = gcode->get_value('X'+i);
                                    if(gcode->subcode == 0) {
                                        this->max_speeds[i] = v;
                                    }
                                    else {
                                        if(gcode->subcode == 1) {
                                            actuators[i]->set_max_rate(v * DEG2RAD);
                                        }
                                    }
                                }
                            }
                            for (size_t i = A_AXIS; i < n_motors; i++) {
                                char c= 'A' + i - A_AXIS;
                                if(gcode->has_letter(c)) {
                                    float v= gcode->get_value(c);
                                    actuators[i]->set_max_rate(v);
                                }
                            }
                            check_max_actuator_speeds();
                        }
                        break;

                case 204: // M204 Snnn - set default acceleration to nnn, Xnnn Ynnn Znnn sets axis specific acceleration
                    if (gcode->has_letter('S')) {
                        float acc = gcode->get_value('S'); // mm/s^2
                        // enforce minimum
                        if (acc < 1.0F) acc = 1.0F;
                        this->default_acceleration = acc;
                    }
                    for (int i = 0; i < n_motors; ++i) {
                        char axis= (i <= Z_AXIS ? 'X'+i : 'A'+(i-A_AXIS));
                        if(gcode->has_letter(axis)) {
                            float acc = gcode->get_value(axis); // mm/s^2
                            // enforce positive
                            if (acc < 1.0F && i > Y_AXIS) acc = NAN;
                            actuators[i]->set_acceleration(acc);
                        }
                    }
                    break;

                case 205: // M205 Xnnn - set junction deviation, Z - set Z junction deviation, Snnn - Set minimum planner speed
                    if (gcode->has_letter('X')) {
                        float jd = gcode->get_value('X');
                        // enforce minimum
                        if (jd < 0.0F)
                            jd = 0.0F;
                        THEKERNEL->planner->junction_deviation = jd;
                    }
                    if (gcode->has_letter('Z')) {
                        float jd = gcode->get_value('Z');
                        // enforce minimum, -1 disables it and uses regular junction deviation
                        if (jd <= -1.0F)
                            jd = NAN;
                        THEKERNEL->planner->z_junction_deviation = jd;
                    }
                    if (gcode->has_letter('S')) {
                        float mps = gcode->get_value('S');
                        // enforce minimum
                        if (mps < 0.0F)
                            mps = 0.0F;
                        THEKERNEL->planner->minimum_planner_speed = mps;
                    }
                    break;

                case 220: // M220 - speed override percentage
                    if (gcode->has_letter('S')) {
                        float factor = gcode->get_value('S');
                        // enforce minimum 10% speed
                        if (factor < 10.0F)
                            factor = 10.0F;
                        // enforce maximum 10x speed
                        if (factor > 1000.0F)
                            factor = 1000.0F;

                        seconds_per_minute = 6000.0F / factor;
                    } else {
                        gcode->stream->printf("Speed factor at %6.2f %%\n", 6000.0F / seconds_per_minute);
                    }
                    break;

                case 400: // wait until all moves are done up to this point
                    THEKERNEL->conveyor->wait_for_idle();
                    break;

                case 500: // M500 saves some volatile settings to config override file
                case 503: // M503 just prints the settings
                    gcode->stream->printf(";Steps per unit:\nM92 ");
                    for (int i = 0; i < n_motors; ++i) {
                        char axis= (i <= Z_AXIS ? 'X'+i : 'A'+(i-A_AXIS));
                        gcode->stream->printf("%c%1.5f ", axis, actuators[i]->get_steps_per_mm());
                    }
                    gcode->stream->printf("\n");

                    // only print if not NAN
                    gcode->stream->printf(";Acceleration mm/sec^2:\nM204 S%1.5f ", default_acceleration);
                    for (int i = 0; i < n_motors; ++i) {
                        char axis= (i <= Z_AXIS ? 'X'+i : 'A'+(i-A_AXIS));
                        if(!isnan(actuators[i]->get_acceleration())) gcode->stream->printf("%c%1.5f ", axis, actuators[i]->get_acceleration());
                    }
                    gcode->stream->printf("\n");

                    gcode->stream->printf(";X- Junction Deviation, Z- Z junction deviation, S - Minimum Planner speed mm/sec:\nM205 X%1.5f Z%1.5f S%1.5f\n", THEKERNEL->planner->junction_deviation, isnan(THEKERNEL->planner->z_junction_deviation)?-1:THEKERNEL->planner->z_junction_deviation, THEKERNEL->planner->minimum_planner_speed);

                    gcode->stream->printf(";Max cartesian feedrates in mm/sec:\nM203 X%1.5f Y%1.5f Z%1.5f S%1.5f\n", this->max_speeds[X_AXIS], this->max_speeds[Y_AXIS], this->max_speeds[Z_AXIS], this->max_speed);

                    gcode->stream->printf(";Max actuator feedrates in mm/sec:\nM203.1 ");
                    for (int i = 0; i < n_motors; ++i) {
                        char axis= (i <= Z_AXIS ? 'X'+i : 'A'+(i-A_AXIS));
                        gcode->stream->printf("%c%1.5f ", axis, actuators[i]->get_max_rate());
                    }
                    gcode->stream->printf("\n");

                    // get or save any arm solution specific optional values
                    BaseSolution::arm_options_t options;
                    if(arm_solution->get_optional(options) && !options.empty()) {
                        gcode->stream->printf(";Optional arm solution specific settings:\nM665");
                        for(auto &i : options) {
                            gcode->stream->printf(" %c%1.4f", i.first, i.second);
                        }
                        gcode->stream->printf("\n");
                    }
                    break;
            }   // switch( gcode->m )
        }   // if( gcode->has_m)
    }   // else ( gcode->has_g)

    if( motion_mode != NONE) {
        is_g123= motion_mode != SEEK;
        process_move(gcode, motion_mode);
    }else{
        is_g123= false;
    }
}

// process a G0/G1/G2/G3
void Robot::process_move(Gcode *gcode, enum MOTION_MODE_T motion_mode)
{
    // we have a G0/G1/G2/G3
    // get XYZ
    float param[4]{NAN, NAN, NAN, NAN};
    bool galvo_move = false;

    // process primary axis
    if( gcode->has_letter('X') ) {
        param[X_AXIS] = gcode->get_value('X');
        galvo_move = true;
    }
    if( gcode->has_letter('Y') ) {
        param[Y_AXIS] = gcode->get_value('Y');
        galvo_move = true;
    }

    // if X or Y is in this gcode, all other axes will be ignored!
    // They are not allowed to move at the same time
    float target[n_motors];
    memcpy(target, machine_position, n_motors*sizeof(float));
    if(galvo_move) {
        if(this->absolute_mode) {
            if(!isnan(param[X_AXIS])) target[X_AXIS] = param[X_AXIS];
            if(!isnan(param[Y_AXIS])) target[Y_AXIS] = param[Y_AXIS];
        }else{
            // they are deltas from the machine_position if specified
            if(!isnan(param[X_AXIS])) target[X_AXIS] = param[X_AXIS] + machine_position[X_AXIS];
            if(!isnan(param[Y_AXIS])) target[Y_AXIS] = param[Y_AXIS] + machine_position[Y_AXIS];
        }
    }
    else {
        // ABC will be ignored if XY or Z is in this gcode
        if( gcode->has_letter('Z') ) {
            param[Z_AXIS] = gcode->get_value('Z');
            if(this->absolute_mode) {
                target[Z_AXIS]= gcode->get_value('Z');
            }
            else{
                target[Z_AXIS]= gcode->get_value('Z') + machine_position[Z_AXIS];
            }
        }
        else {
            // process ABC axis
            for (int i = A_AXIS; i < n_motors; ++i) {
                char letter = 'A' + i - A_AXIS;
                if(gcode->has_letter(letter)) {
                    if(this->absolute_mode) {
                        target[i]= gcode->get_value(letter);
                    }else{
                        target[i]= gcode->get_value(letter) + machine_position[i];
                    }
                }
            }
        }
    }
    // S is modal When specified on a G0/1/2/3 command
//    if(gcode->has_letter('S')) s_value= gcode->get_value('S');
    
    bool moved= false;

    // Perform any physical actions
    if(motion_mode == LINEAR) {
        if( gcode->has_letter('F') ) this->feed_rate = gcode->get_value('F');
        moved= this->append_line(gcode, target, this->feed_rate / seconds_per_minute, galvo_move);
    }
    else if(motion_mode == SEEK) {
        if( gcode->has_letter('F') ) this->seek_rate = gcode->get_value('F');
        moved= this->append_line(gcode, target, this->seek_rate / seconds_per_minute, galvo_move);
    }

    if(moved) {
        // set machine_position to the calculated target
        memcpy(machine_position, target, n_motors*sizeof(float));
    }
}

// Append a move to the queue ( cutting it into segments if needed )
bool Robot::append_line(Gcode *gcode, const float target[], float rate_mm_s, bool galvo_move)
{
    // catch negative or zero feed rates and return the same error as GRBL does
    if(rate_mm_s <= 1.0F) {
        gcode->is_error= true;
        gcode->txt_after_ok= (rate_mm_s == 0 ? "Undefined feed rate" : "feed rate < 0");
        return false;
    }

    bool moved= false;
    // Find out the distance for this move in XYZ
    if(galvo_move) {
        float millimeters_of_travel = sqrtf(powf( target[X_AXIS] - machine_position[X_AXIS], 2 ) +  powf( target[Y_AXIS] - machine_position[Y_AXIS], 2 ) );

        if(millimeters_of_travel > 0.001F) {
            // We cut the line into smaller segments. This is only needed on a cartesian robot for zgrid, but always necessary for robots with rotational axes like Deltas.
            // In delta robots either mm_per_line_segment can be used OR delta_segments_per_second
            // The latter is more efficient and avoids splitting fast long lines into very small segments, like initial z move to 0, it is what Johanns Marlin delta port does
            uint16_t segments;

            if(this->mm_per_line_segment > 0.01F && gcode->g == 1) {
                segments = ceilf( millimeters_of_travel / this->mm_per_line_segment);
            }
            else {segments= 1;}

            if (segments > 1) {
                // A vector to keep track of the endpoint of each segment
                float segment_delta[N_PRIMARY_AXIS];
                float segment_end[n_motors];
                memcpy(segment_end, machine_position, n_motors*sizeof(float));

                // How far do we move each segment?
                segment_delta[X_AXIS] = (target[X_AXIS] - machine_position[X_AXIS]) / segments;
                segment_delta[Y_AXIS] = (target[Y_AXIS] - machine_position[Y_AXIS]) / segments;

                // segment 0 is already done - it's the end point of the previous move so we start at segment 1
                // We always add another point after this loop so we stop at segments-1, ie i < segments
                for (int i = 1; i < segments; i++) {
                    if(THEKERNEL->is_halted()) return false; // don't queue any more segments

                    segment_end[X_AXIS] += segment_delta[X_AXIS];
                    segment_end[Y_AXIS] += segment_delta[Y_AXIS];

                    // Append the end of this segment to the queue
                    // this can block waiting for free block queue or if in feed hold
                    bool b= this->append_milestone(segment_end, rate_mm_s, galvo_move);
                    moved= moved || b;  //in case some segment got rejected
                }
            }
        }
    }

    // Append the end of this full move to the queue
    if(this->append_milestone(target, rate_mm_s, galvo_move)) moved= true;
    
    return moved;
}

//#include "gpio.h"
//extern GPIO leds[];
// Convert target (in machine coordinates) to machine_position, then convert to actuator position and append this to the planner
// target is in machine coordinates
bool Robot::append_milestone(const float target[], float rate_mm_s, bool galvo_move)
{
    float deltas[n_motors];
    float am_target[n_motors]; // adjust target for bed compensation
    float sos= 0; // sum of squares for just primary axis (XYZ usually)
    float distance;
    // unity transform by default
    memcpy(am_target, target, n_motors*sizeof(float));

    bool move= false;
    if(galvo_move) {
        deltas[X_AXIS] = am_target[X_AXIS] - machine_position[X_AXIS];
        deltas[Y_AXIS] = am_target[Y_AXIS] - machine_position[Y_AXIS];
        if(fabsf(deltas[X_AXIS]) > 0.005F) {
            sos += powf(deltas[X_AXIS], 2);
            move = true;
        }
        if(fabsf(deltas[Y_AXIS]) > 0.005F) {
            sos += powf(deltas[Y_AXIS], 2);
            move = true;
        }
        distance= sqrtf(sos);
//        if(distance < 0.001F) return false;
        if( rate_mm_s > this->max_speed ) {
            rate_mm_s = this->max_speed;
        }
    }
    else {
        for (size_t i = Z_AXIS; i < n_motors; i++) {
            deltas[i] = am_target[i] - machine_position[i];
            if(fabsf(deltas[i]) < 0.001F) continue;
            // at least one non zero delta
            move = true;
            distance = fabsf(deltas[i]);
            break;
        }
    }
    // nothing moved
    if(!move) return false;

    // total movement, use XY if a primary axis otherwise we calculate distance
    ActuatorCoordinates actuator_pos;

//    DEBUG_PRINTF("distance: %f, aux_move: %d\n", distance, auxilliary_move);

    // use default acceleration to start with
    float acceleration = default_acceleration;

//    float isecs = rate_mm_s / distance;

    // check per-actuator speed limits
    if(galvo_move) {
        // find actuator position given the machine position.
        arm_solution->cartesian_to_actuator( am_target, actuator_pos );

//        for (size_t actuator = X_AXIS; actuator <= Y_AXIS ; actuator++) {
//            float d = fabsf(actuator_pos[actuator] - actuators[actuator]->get_last_milestone());
//            if(d < 0.00001F) continue;// no realistic movement for this actuator

//            float actuator_rate= d * isecs;
//            if (actuator_rate > actuators[actuator]->get_max_rate()) {
//                rate_mm_s *= (actuators[actuator]->get_max_rate() / actuator_rate);
//            }
//        }
//        acceleration = actuators[0]->get_acceleration();
    }
    else {
        for (size_t i = Z_AXIS; i < n_motors; i++) {
            actuator_pos[i] = am_target[i];
            if (rate_mm_s > actuators[i]->get_max_rate()) {
                rate_mm_s = actuators[i]->get_max_rate();
            }
            float ma = actuators[i]->get_acceleration(); // in mm/sec²
            if(!isnan(ma)) {  // if axis does not have acceleration set then it uses the default_acceleration
                acceleration = ma;
            }
        }
    }

    // Append the block to the planner
    // NOTE that distance here should be either the distance travelled by the XYZ axis, or the E mm travel if a solo E move
    // NOTE this call will bock until there is room in the block queue, on_idle will continue to be called
    if(THEKERNEL->planner->append_block( actuator_pos, n_motors, rate_mm_s, distance, acceleration, galvo_move, is_g123)) {
        // this is the new machine position
        memcpy(this->machine_position, am_target, n_motors*sizeof(float));
        return true;
    }

    // no actual move, should never happen
    return false;
}

// Reset the position for an axis (used in homing after suspend)
void Robot::reset_axis_position(float position, int axis)
{
    if(axis >= Z_AXIS)
        actuators[axis]->change_last_milestone(position);
}

// Use FK to find out where actuator is and reset to match
// TODO maybe we should only reset axis that are being homed unless this is due to a ON_HALT
void Robot::reset_position_from_current_actuator_position()
{
    ActuatorCoordinates actuator_pos;
//    #if MAX_ROBOT_ACTUATORS > 2
    for (int i = Z_AXIS; i < n_motors; i++) {
        actuator_pos[i] = actuators[i]->get_current_position();
        machine_position[i]= actuator_pos[i];
        actuators[i]->change_last_milestone(actuator_pos[i]); // this updates the last_milestone in the actuator
    }
//    #endif
}

// Used to plan a single move used by things like endstops when homing
bool Robot::delta_move(const float *delta, float rate_mm_s, uint8_t naxis)
{
    if(THEKERNEL->is_halted()) return false;

    // catch negative or zero feed rates
    if(rate_mm_s <= 0.0F) {
        return false;
    }

    // get the absolute target position, default is current machine_position
    float target[n_motors];
    memcpy(target, machine_position, n_motors*sizeof(float));

    // add in the deltas to get new target
    for (int i= 0; i < naxis; i++) {
        target[i] += delta[i];
    }

    is_g123= false; // we don't want the laser to fire
    // submit for planning and if moved update machine_position
    if(append_milestone(target, rate_mm_s, false)) {
         memcpy(machine_position, target, n_motors*sizeof(float));
         return true;
    }

    return false;
}

float Robot::theta(float x, float y)
{
    float t = atanf(x / fabs(y));
    if (y > 0) {
        return(t);
    } else {
        if (t > 0) {
            return(PI - t);
        } else {
            return(-PI - t);
        }
    }
}

float Robot::get_feed_rate() const
{
    return THEKERNEL->gcode_dispatch->get_modal_command() == 0 ? seek_rate : feed_rate;
}

bool Robot::is_homed(uint8_t i) const
{
    if(i >= 3) return false; // safety

    // if we are homing we ignore soft endstops so return false
    bool homing;
    bool ok = PublicData::get_value(endstops_checksum, get_homing_status_checksum, 0, &homing);
    if(!ok || homing) return false;

    // check individual axis homing status
    bool homed[3];
    ok = PublicData::get_value(endstops_checksum, get_homed_status_checksum, 0, homed);
    if(!ok) return false;
    return homed[i];
}
