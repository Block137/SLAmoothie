/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "modules/communication/utils/Gcode.h"
#include "modules/robot/Conveyor.h"
#include "modules/robot/ActuatorCoordinates.h"
#include "Endstops.h"
#include "libs/nuts_bolts.h"
#include "libs/Pin.h"
#include "libs/StepperMotor.h"
#include "wait_api.h" // mbed.h lib
#include "Robot.h"
#include "Config.h"
#include "SlowTicker.h"
#include "Planner.h"
#include "checksumm.h"
#include "utils.h"
#include "ConfigValue.h"
#include "libs/StreamOutput.h"
#include "PublicDataRequest.h"
#include "EndstopsPublicAccess.h"
#include "StreamOutputPool.h"
#include "StepTicker.h"
#include "BaseSolution.h"
#include "SerialMessage.h"

#include <ctype.h>
#include <algorithm>

// OLD deprecated syntax
#define endstops_module_enable_checksum         CHECKSUM("endstops_enable")

#define ENDSTOP_CHECKSUMS(X) {            \
    CHECKSUM(X "_min_endstop"),           \
    CHECKSUM(X "_max_endstop"),           \
    CHECKSUM(X "_max_travel"),            \
    CHECKSUM(X "_fast_homing_rate_mm_s"), \
    CHECKSUM(X "_slow_homing_rate_mm_s"), \
    CHECKSUM(X "_homing_retract_mm"),     \
    CHECKSUM(X "_homing_direction"),      \
    CHECKSUM(X "_min"),                   \
    CHECKSUM(X "_max"),                   \
    CHECKSUM(X "_limit_enable"),          \
}

// checksum defns
enum DEFNS {MIN_PIN, MAX_PIN, MAX_TRAVEL, FAST_RATE, SLOW_RATE, RETRACT, DIRECTION, MIN, MAX, LIMIT, NDEFNS};

// global config settings
#define corexy_homing_checksum           CHECKSUM("corexy_homing")
#define delta_homing_checksum            CHECKSUM("delta_homing")
#define rdelta_homing_checksum           CHECKSUM("rdelta_homing")
#define scara_homing_checksum            CHECKSUM("scara_homing")

#define endstop_debounce_count_checksum  CHECKSUM("endstop_debounce_count")
#define endstop_debounce_ms_checksum     CHECKSUM("endstop_debounce_ms")

#define home_z_first_checksum            CHECKSUM("home_z_first")
#define homing_order_checksum            CHECKSUM("homing_order")
#define move_to_origin_checksum          CHECKSUM("move_to_origin_after_home")
#define park_after_home_checksum         CHECKSUM("park_after_home")

#define alpha_trim_checksum              CHECKSUM("alpha_trim_mm")
#define beta_trim_checksum               CHECKSUM("beta_trim_mm")
#define gamma_trim_checksum              CHECKSUM("gamma_trim_mm")

// new config syntax
// endstop.xmin.enable true
// endstop.xmin.pin 1.29
// endstop.xmin.axis X
// endstop.xmin.homing_direction home_to_min

#define endstop_checksum                   CHECKSUM("endstop")
#define enable_checksum                    CHECKSUM("enable")
#define pin_checksum                       CHECKSUM("pin")
#define axis_checksum                      CHECKSUM("axis")
#define direction_checksum                 CHECKSUM("homing_direction")
#define position_checksum                  CHECKSUM("homing_position")
#define fast_rate_checksum                 CHECKSUM("fast_rate")
#define slow_rate_checksum                 CHECKSUM("slow_rate")
#define max_travel_checksum                CHECKSUM("max_travel")
#define retract_checksum                   CHECKSUM("retract")
#define limit_checksum                     CHECKSUM("limit_enable")

#define STEPPER THEROBOT->actuators
#define STEPS_PER_MM(a) (STEPPER[a]->get_steps_per_mm())



// Homing States
enum STATES {
    MOVING_TO_ENDSTOP_FAST, // homing move
    MOVING_TO_ENDSTOP_SLOW, // homing move
    MOVING_BACK,            // homing move
    NOT_HOMING,
    BACK_OFF_HOME,
    MOVE_TO_ORIGIN,
    LIMIT_TRIGGERED
};

Endstops::Endstops()
{
    this->status = NOT_HOMING;
}

void Endstops::on_module_loaded()
{
    // Do not do anything if not enabled or if no pins are defined
    if (THEKERNEL->config->value( endstops_module_enable_checksum )->by_default(false)->as_bool()) {
        delete this;
        return;

    }else{
        // check for new config syntax
        if(!load_config()) {
            delete this;
            return;
        }
    }

    register_for_event(ON_GCODE_RECEIVED);
    register_for_event(ON_GET_PUBLIC_DATA);

    THEKERNEL->slow_ticker->attach(1000, this, &Endstops::read_endstops);
}

// Get config using new syntax supports ABC
bool Endstops::load_config()
{
    bool limit_enabled= false;
    size_t max_index= 0;

    std::array<homing_info_t, k_max_actuators> temp_axis_array; // needs to be at least XYZ, but allow for ABC
    {
        homing_info_t t;
        t.axis= 0;
        t.axis_index= 0;
        t.pin_info= nullptr;

        temp_axis_array.fill(t);
    }

    // iterate over all endstop.*.*
    std::vector<uint16_t> modules;
    THEKERNEL->config->get_module_list(&modules, endstop_checksum);
    for(auto cs : modules ) {
        if(!THEKERNEL->config->value(endstop_checksum, cs, enable_checksum )->as_bool()) continue;

        endstop_info_t *pin_info= new endstop_info_t;
        pin_info->pin.from_string(THEKERNEL->config->value(endstop_checksum, cs, pin_checksum)->by_default("nc" )->as_string())->as_input();
        if(!pin_info->pin.connected()){
            // no pin defined try next
            delete pin_info;
            continue;
        }

        string axis= THEKERNEL->config->value(endstop_checksum, cs, axis_checksum)->by_default("")->as_string();
        if(axis.empty()){
            // axis is required
            delete pin_info;
            continue;
        }

        size_t i;
        switch(toupper(axis[0])) {
//            case 'X': i= X_AXIS; break;
//            case 'Y': i= Y_AXIS; break;
            case 'Z': i= Z_AXIS; break;
            case 'A': i= A_AXIS; break;
            case 'B': i= B_AXIS; break;
            case 'C': i= C_AXIS; break;
            default: // not a recognized axis
                delete pin_info;
                continue;
        }

        // check we are not going above the number of defined actuators/axis
        if(i >= THEROBOT->get_number_registered_motors()) {
            // too many axis we only have configured n_motors
            THEKERNEL->streams->printf("ERROR: endstop %d is greater than number of defined motors. Endstops disabled\n", i);
            delete pin_info;
            return false;
        }

        // keep track of the maximum index that has been defined
        if(i > max_index) max_index= i;

        // init pin struct
        pin_info->debounce= 0;
        pin_info->axis= toupper(axis[0]);
        pin_info->axis_index= i;

        // are limits enabled
        pin_info->limit_enable= THEKERNEL->config->value(endstop_checksum, cs, limit_checksum)->by_default(false)->as_bool();
        limit_enabled |= pin_info->limit_enable;

        // enter into endstop array
        endstops.push_back(pin_info);

        // if set to none it means not used for homing (maybe limit only) so do not add to the homing array
        string direction= THEKERNEL->config->value(endstop_checksum, cs, direction_checksum)->by_default("none")->as_string();
        if(direction == "none") {
            continue;
        }

        // setup the homing array
        homing_info_t hinfo;

        // init homing struct
        hinfo.home_offset= 0;
        hinfo.homed= false;
        hinfo.axis= toupper(axis[0]);
        hinfo.axis_index= i;
        hinfo.pin_info= pin_info;

        // rates in mm/sec
        hinfo.fast_rate= THEKERNEL->config->value(endstop_checksum, cs, fast_rate_checksum)->by_default(100)->as_number();
        hinfo.slow_rate= THEKERNEL->config->value(endstop_checksum, cs, slow_rate_checksum)->by_default(10)->as_number();

        // retract in mm
        hinfo.retract= THEKERNEL->config->value(endstop_checksum, cs, retract_checksum)->by_default(5)->as_number();

        // homing direction and convert to boolean where true is home to min, and false is home to max
        hinfo.home_direction=  direction == "home_to_min";

        // homing cartesian position
        hinfo.homing_position= THEKERNEL->config->value(endstop_checksum, cs, position_checksum)->by_default(hinfo.home_direction ? 0 : 200)->as_number();

        // used to set maximum movement on homing, set by max_travel if defined
        hinfo.max_travel= THEKERNEL->config->value(endstop_checksum, cs, max_travel_checksum)->by_default(500)->as_number();

        // stick into array in correct place
        temp_axis_array[hinfo.axis_index]= hinfo;
    }

    // if no pins defined then disable the module
    if(endstops.empty()) return false;

    // copy to the homing_axis array, make sure that undefined entries are filled in as well
    // as the order is important and all slots must be filled upto the max_index
    for (size_t i = 0; i < temp_axis_array.size(); ++i) {
        if(temp_axis_array[i].axis == 0) {
            // was not configured above, if it is XYZ then we need to force a dummy entry
            if(i <= Z_AXIS) {
                homing_info_t t;
                t.axis= 'X' + i;
                t.axis_index= i;
                t.pin_info= nullptr; // this tells it that it cannot be used for homing
                homing_axis.push_back(t);

            }else if(i <= max_index) {
                // for instance case where we defined C without A or B
                homing_info_t t;
                t.axis= 'A' + i;
                t.axis_index= i;
                t.pin_info= nullptr; // this tells it that it cannot be used for homing
                homing_axis.push_back(t);
            }

        }else{
            homing_axis.push_back(temp_axis_array[i]);
        }
    }

    // saves some memory
    homing_axis.shrink_to_fit();
    endstops.shrink_to_fit();

    // sets some endstop global configs applicable to all endstops
    get_global_configs();

    return true;
}

void Endstops::get_global_configs()
{
    // NOTE the debounce count is in milliseconds so probably does not need to beset anymore
    this->debounce_ms= THEKERNEL->config->value(endstop_debounce_ms_checksum)->by_default(0)->as_number();
    this->debounce_count= THEKERNEL->config->value(endstop_debounce_count_checksum)->by_default(100)->as_number();

    this->home_z_first= THEKERNEL->config->value(home_z_first_checksum)->by_default(false)->as_bool();

    // see if an order has been specified, must be three or more characters, XYZABC or ABYXZ etc
    string order = THEKERNEL->config->value(homing_order_checksum)->by_default("")->as_string();
    this->homing_order = 0;
    if(order.size() >= 3 && order.size() <= homing_axis.size() ) {
        int shift = 0;
        for(auto c : order) {
            char n= toupper(c);
            uint32_t i = n >= 'X' ? n - 'X' : n - 'A' + 3;
            i += 1; // So X is 1
            if(i > 6) { // bad value
                this->homing_order = 0;
                break;
            }
            homing_order |= (i << shift);
            shift += 3;
        }
    }
}

bool Endstops::debounced_get(Pin *pin)
{
    if(pin == nullptr) return false;
    uint32_t debounce = 0;
    while(pin->get()) {
        if ( ++debounce >= this->debounce_count ) {
            // pin triggered
            return true;
        }
    }
    return false;
}

// Called every millisecond in an ISR
uint32_t Endstops::read_endstops(uint32_t dummy)
{
    if(this->status != MOVING_TO_ENDSTOP_SLOW && this->status != MOVING_TO_ENDSTOP_FAST) return 0; // not doing anything we need to monitor for

    // check each homing endstop
    for(auto& e : homing_axis) { // check all axis homing endstops
        if(e.pin_info == nullptr) continue; // ignore if not a homing endstop
        int m= e.axis_index;

        if(STEPPER[m]->is_moving()) {
            // if it is moving then we check the associated endstop, and debounce it
            if(e.pin_info->pin.get()) {
                if(e.pin_info->debounce < debounce_ms) {
                    e.pin_info->debounce++;

                } else {
                    // we signal the motor to stop, which will preempt any moves on that axis
                    STEPPER[m]->stop_moving();
                    e.pin_info->triggered= true;
                }
            } else {
                // The endstop was not hit yet
                e.pin_info->debounce= 0;
            }
        }
    }

    return 0;
}

void Endstops::home(axis_bitmap_t a)
{
    // reset debounce counts for all endstops
    for(auto& e : endstops) {
       e->debounce= 0;
       e->triggered= false;
    }

    this->axis_to_home= a;

    // Start moving the axes to the origin
    this->status = MOVING_TO_ENDSTOP_FAST;

    // potentially home A B and C individually
    if(homing_axis.size() > 2){
        for (size_t i = Z_AXIS; i < homing_axis.size(); ++i) {
            if(axis_to_home[i]) {
                // now home A B or C
                float delta[i+1];
                for (size_t j = 0; j <= i; ++j) delta[j]= 0;
                delta[i]= homing_axis[i].max_travel; // we go the max
                if(homing_axis[i].home_direction) delta[i]= -delta[i];
                THEROBOT->delta_move(delta, homing_axis[i].fast_rate, i+1);
                // wait for it
                THECONVEYOR->wait_for_idle();
            }
        }
    }

    // check that the endstops were hit and it did not stop short for some reason
    // if the endstop is not triggered then enter ALARM state
    // with deltas we check all three axis were triggered, but at least one of XYZ must be set to home
    // also check ABC
    if(homing_axis.size() > 2){
        for (size_t i = Z_AXIS; i < homing_axis.size(); ++i) {
            if(axis_to_home[i] && !homing_axis[i].pin_info->triggered) {
                this->status = NOT_HOMING;
                THEKERNEL->call_event(ON_HALT, nullptr);
                return;
            }
        }
    }

    // we did not complete movement the full distance if we hit the endstops
    // TODO Maybe only reset axis involved in the homing cycle
    THEROBOT->reset_position_from_current_actuator_position();

    // Move back a small distance for all homing axis
    this->status = MOVING_BACK;
    float delta[homing_axis.size()];
    for (size_t i = 0; i < homing_axis.size(); ++i) delta[i]= 0;

    // use minimum feed rate of all axes that are being homed (sub optimal, but necessary)
    for (auto& i : homing_axis) {
        int c= i.axis_index;
        if(axis_to_home[c]) {
            delta[c]= i.retract;
            if(!i.home_direction) {delta[c]= -delta[c];}
            THEROBOT->delta_move(delta, i.slow_rate, homing_axis.size());
            // wait until finished
            THECONVEYOR->wait_for_idle();
        }
    }

    // Start moving the axes towards the endstops slowly
    this->status = MOVING_TO_ENDSTOP_SLOW;
    for (size_t i = 0; i < homing_axis.size(); ++i) delta[i]= 0;
    for (auto& i : homing_axis) {
        int c= i.axis_index;
        if(axis_to_home[c]) {
            delta[c]= i.retract*2; // move further than we moved off to make sure we hit it cleanly
            if(i.home_direction) delta[c]= -delta[c];
            THEROBOT->delta_move(delta, i.slow_rate, homing_axis.size());
            // wait until finished
            THECONVEYOR->wait_for_idle();
        }
    }

    // we did not complete movement the full distance if we hit the endstops
    // TODO Maybe only reset axis involved in the homing cycle
    THEROBOT->reset_position_from_current_actuator_position();

    this->status = NOT_HOMING;
}

void Endstops::process_home_command(Gcode* gcode)
{
    // First wait for the queue to be empty
    THECONVEYOR->wait_for_idle();

    // figure out which axis to home
    axis_bitmap_t haxis;
    haxis.reset();

    bool axis_speced = (gcode->has_letter('X') || gcode->has_letter('Y') || gcode->has_letter('Z') ||
                        gcode->has_letter('A') || gcode->has_letter('B') || gcode->has_letter('C'));

    for (auto &p : homing_axis) {
        // only enable homing if the endstop is defined,
        if(p.pin_info == nullptr) continue;
        if(!axis_speced || gcode->has_letter(p.axis)) {
            haxis.set(p.axis_index);
            // now reset axis to 0 as we do not know what state we are in
            THEROBOT->reset_axis_position(0, p.axis_index);
        }
    }

    if(haxis.none()) {
        THEKERNEL->streams->printf("WARNING: Nothing to home\n");
        return;
    }

    // do the actual homing
    if(homing_order != 0) {
        // if an order has been specified do it in the specified order
        // homing order is 0bfffeeedddcccbbbaaa where aaa is 1,2,3,4,5,6 to specify the first axis (XYZABC), bbb is the second and ccc is the third etc
        // eg 0b0101011001010 would be Y X Z A, 011 010 001 100 101 would be  B A X Y Z
        for (uint32_t m = homing_order; m != 0; m >>= 3) {
            uint32_t a= (m & 0x07)-1; // axis to home
            if(a < homing_axis.size() && haxis[a]) { // if axis is selected to home
                axis_bitmap_t bs;
                bs.set(a);
                home(bs);
            }
            // check if on_halt (eg kill)
            if(THEKERNEL->is_halted()) break;
        }
    } else {
        // they could all home at the same time
        home(haxis);
    }

    // check if on_halt (eg kill or fail)
    if(THEKERNEL->is_halted()) {
        THEKERNEL->streams->printf("ERROR: Homing failed\n");
        // clear all the homed flags
        for (auto &p : homing_axis) p.homed= false;
        return;
    }

    // Zero the ax(i/e)s position
    for (auto &p : homing_axis) {
        if (haxis[p.axis_index]) { // if we requested this axis to home
            THEROBOT->reset_axis_position(p.homing_position, p.axis_index);
            // set flag indicating axis was homed, it stays set once set until H/W reset or unhomed
            p.homed= true;
        }
    }
}

// parse gcodes
void Endstops::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    if ( gcode->has_g && gcode->g == 28) {
        process_home_command(gcode);
    } else {
        if (gcode->has_m && gcode->m == 119) {

        for(auto& h : homing_axis) {
            if(h.pin_info == nullptr) continue; // ignore if not a homing endstop
            string name;
            name.append(1, h.axis).append(h.home_direction ? "_min" : "_max");
            gcode->stream->printf("%s:%d ", name.c_str(), h.pin_info->pin.get());
        }
        gcode->stream->printf("pins- ");
        for(auto& p : endstops) {
            string str(1, p->axis);
            if(p->limit_enable) str.append("L");
            gcode->stream->printf("(%s)P%d.%d:%d ", str.c_str(), p->pin.port_number, p->pin.pin, p->pin.get());
        }
        gcode->add_nl = true;
        }
    }
}

void Endstops::on_get_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(endstops_checksum)) return;

    if(pdr->second_element_is(get_homing_status_checksum)) {
        bool *homing = static_cast<bool *>(pdr->get_data_ptr());
        *homing = this->status != NOT_HOMING;
        pdr->set_taken();

    } else if(pdr->second_element_is(get_homed_status_checksum)) {
        bool *homed = static_cast<bool *>(pdr->get_data_ptr());
        for (int i = 0; i < 3; ++i) {
            homed[i]= homing_axis[i].homed;
        }
        pdr->set_taken();
    }
}