/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl) with additions from Sungeun K. Jeon (https://github.com/chamnit/grbl)
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

using namespace std;

#include "mri.h"
#include "nuts_bolts.h"
#include "RingBuffer.h"
#include "Gcode.h"
#include "Module.h"
#include "Kernel.h"
#include "Block.h"
#include "Planner.h"
#include "Conveyor.h"
#include "StepperMotor.h"
#include "Config.h"
#include "checksumm.h"
#include "Robot.h"
#include "ConfigValue.h"

#include <math.h>
#include <algorithm>

#define junction_deviation_checksum    CHECKSUM("junction_deviation")
#define z_junction_deviation_checksum  CHECKSUM("z_junction_deviation")
#define minimum_planner_speed_checksum CHECKSUM("minimum_planner_speed")

// The Planner does the acceleration math for the queue of Blocks ( movements ).
// It makes sure the speed stays within the configured constraints ( acceleration, junction_deviation, etc )
// It goes over the list in both direction, every time a block is added, re-doing the math to make sure everything is optimal

Planner::Planner()
{
    config_load();
}

// Configure acceleration
void Planner::config_load()
{
    this->junction_deviation    = THEKERNEL->config->value(junction_deviation_checksum)->by_default(0.0F)->as_number();
    this->z_junction_deviation  = THEKERNEL->config->value(z_junction_deviation_checksum)->by_default(NAN)->as_number(); // disabled by default
    this->minimum_planner_speed = THEKERNEL->config->value(minimum_planner_speed_checksum)->by_default(0.0f)->as_number();
//    this->DAC_neutral           = THEKERNEL->config->value(dac_neutral_checksum)->by_default(32767)->as_number();
}


// Append a block to the queue, compute it's speed factors
bool Planner::append_block( ActuatorCoordinates &actuator_pos, uint8_t n_motors, float rate_mm_s, float distance, float acceleration, bool galvo_move, bool g123)
{
    // Create ( recycle ) a new block
    Block* block = THECONVEYOR->queue.head_ref();

    // use either regular junction deviation or z specific and see if a primary axis move
    block->primary_axis = galvo_move;

    // Direction bits
    bool has_steps = false;
    int32_t steps = 0;
    if(galvo_move) {   //this is galvo move
        for (size_t i = 0; i < N_PRIMARY_AXIS; i++) {
            steps = THEROBOT->actuators[i]->steps_to_target(actuator_pos[i]);
            if(steps != 0) {
                THEROBOT->actuators[i]->update_last_milestones(actuator_pos[i], steps);
                has_steps = true;
            }
            block->steps[i] = labs(steps);
            block->direction_bits[i] = (steps < 0) ? 1 : 0;
        }
        block->is_g123 = g123;
        // Max number of steps
        block->steps_event_count = block->steps[0] >= block->steps[1] ?  block->steps[0] : block->steps[1];
    }
    else {
        for (size_t i = N_PRIMARY_AXIS; i < n_motors; i++) {
            steps = THEROBOT->actuators[i]->steps_to_target(actuator_pos[i]);
            // Update current position
            if(steps != 0) {
                THEROBOT->actuators[i]->update_last_milestones(actuator_pos[i], steps);
                has_steps = true;
                block->steps[i] = labs(steps);
                block->direction_bits[i] = (steps < 0) ? 1 : 0;
                // Max number of steps, for all axes
                block->steps_event_count = block->steps[i];
            }
            else {  
                // ignore direction
                // save actual steps in block
                block->steps[i] = 0;
            }
        }
        block->is_g123 = false;
        block->acceleration = acceleration; // save in block
    }
    // sometimes even though there is a detectable movement it turns out there are no steps to be had from such a small move
    if(!has_steps) {
        block->clear();
        // we still return true so the tiny move will still be accumulated and eventually create steps
        return true;
    }

    // use default JD
//    float junction_deviation = this->junction_deviation;

    block->millimeters = distance;

    // Calculate speed in mm/sec for each axis. No divide by zero due to previous checks.
    block->nominal_speed = rate_mm_s;           // (mm/s) Always > 0
    block->nominal_rate = block->steps_event_count * rate_mm_s / distance; // (step/s) Always > 0

    // Compute the acceleration rate for the trapezoid generator. Depending on the slope of the line
    // average travel per step event changes. For a line along one axis the travel per step event
    // is equal to the travel/step in the particular axis. For a 45 degree line the steppers of both
    // axes might step for every step event. Travel per step event is then sqrt(travel_x^2+travel_y^2).

    // Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
    // Let a circle be tangent to both previous and current path line segments, where the junction
    // deviation is defined as the distance from the junction to the closest edge of the circle,
    // colinear with the circle center. The circular segment joining the two paths represents the
    // path of centripetal acceleration. Solve for max velocity based on max acceleration about the
    // radius of the circle, defined indirectly by junction deviation. This may be also viewed as
    // path width or max_jerk in the previous grbl version. This approach does not actually deviate
    // from path, but used as a robust way to compute cornering speeds, as it takes into account the
    // nonlinearities of both the junction angle and junction velocity.

    // NOTE however it does not take into account independent axis, in most cartesian X and Y and Z are totally independent
    // and this allows one to stop with little to no decleration in many cases. This is particualrly bad on leadscrew based systems that will skip steps.
//    float vmax_junction = minimum_planner_speed; // Set default max junction speed

//    memset(previous_unit_vec, 0, sizeof(previous_unit_vec));
    block->calculate_trapezoid(galvo_move);

    THECONVEYOR->queue_head_block();
    // The block can now be used
    block->ready();

    return true;
}

/*
void Planner::recalculate()
{
    Conveyor::Queue_t &queue = THECONVEYOR->queue;

    unsigned int block_index;

    Block* previous;
    Block* current;

     // a newly added block is decel limited
     //
     // we find its max entry speed given its exit speed
     //
     // for each block, walking backwards in the queue:
     //
     // if max entry speed == current entry speed
     // then we can set recalculate to false, since clearly adding another block didn't allow us to enter faster
     // and thus we don't need to check entry speed for this block any more
     //
     // once we find an accel limited block, we must find the max exit speed and walk the queue forwards
     //
     // for each block, walking forwards in the queue:
     //
     // given the exit speed of the previous block and our own max entry speed
     // we can tell if we're accel or decel limited (or coasting)
     //
     // if prev_exit > max_entry
     //     then we're still decel limited. update previous trapezoid with our max entry for prev exit
     // if max_entry >= prev_exit
     //     then we're accel limited. set recalculate to false, work out max exit speed
     //
     // finally, work out trapezoid for the final (and newest) block.

     // Step 1:
     // For each block, given the exit speed and acceleration, find the maximum entry speed

    float entry_speed = minimum_planner_speed;

    block_index = queue.head_i;
    current     = queue.item_ref(block_index);

    if (!queue.is_empty()) {
        while ((block_index != queue.tail_i) && current->recalculate_flag) {
            entry_speed = current->reverse_pass(entry_speed);

            block_index = queue.prev(block_index);
            current     = queue.item_ref(block_index);
        }

         // Step 2:
         // now current points to either tail or first non-recalculate block
         // and has not had its reverse_pass called
         // or its calculate_trapezoid
         // entry_speed is set to the *exit* speed of current.
         // each block from current to head has its entry speed set to its max entry speed- limited by decel or nominal_rate

        float exit_speed = current->max_exit_speed();

        while (block_index != queue.head_i) {
            previous    = current;
            block_index = queue.next(block_index);
            current     = queue.item_ref(block_index);

            // we pass the exit speed of the previous block
            // so this block can decide if it's accel or decel limited and update its fields as appropriate
            exit_speed = current->forward_pass(exit_speed);

            previous->calculate_trapezoid(previous->entry_speed, current->entry_speed);
        }
    }

     // Step 3:
     // work out trapezoid for final (and newest) block

    // now current points to the head item
    // which has not had calculate_trapezoid run yet
    current->calculate_trapezoid(current->entry_speed, minimum_planner_speed);
}


// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the
// acceleration within the allotted distance.
float Planner::max_allowable_speed(float acceleration, float target_velocity, float distance)
{
    // Was acceleration*60*60*distance, in case this breaks, but here we prefer to use seconds instead of minutes
    return(sqrtf(target_velocity * target_velocity - 2.0F * acceleration * distance));
}
*/
