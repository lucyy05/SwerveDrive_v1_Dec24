#include "definitions.h"
#include "api.h"

bool ignore_colour = false;
bool is_we_blue_alliance = true;            // are we blue and should therefore score blue
bool is_ring_ours = false;                  // is the ring the same colour as ours (updates after intake, storage and like 500ms)

/* CONVEYOR FUNCTION USAGE
 - run calibrate_conveyor() at ideally once first (ideally, run until the conveyor is slightly higher than the center of the hook)
   - (you could technically skip the calibrate_conveyor() function since a good ish period value has been supplied)
 - step_conveyor() is the function to use to automatically go thru the subroutine to:
   - receive ring (from intake)
   - store (free up intake)
   - score 
   - "calibrate"
 - use conveyor_go_to_absolute() to move conveyor to where you want. 
   Location is supplied by a decimal between 0-1 where 0 is the home position (aka where the hook is first detected by the optical sensor)
   and 1 is the home position for the next hook
 - use conveyor_go_home_by_sensor() to "calibrate" the conveyor position, since it uses the prox sensor, IT IS NOT RELIABLE WHEN IT IS TRANSPORTING A RING
 - use conveyor_go_home() to return to the home position of the conveyor, there might not be much of a need for thsi function.
 - there shouldn't be a need to use _calibrate_at_voltage() outside of the conveyor functions 

CONVEYOR TODO:
 - [/] implement conveyor "proprioception"
 - [/] implement conveyor auto-store from intake
 - [/] implement conveyor colour detection
 - [/] figure out how to put the conveyor functions in another file for organisation 
*/

// #====# conveyor util functions

void bound_conveyor_position(double binding_value){
    if(conveyor.get_position() >= binding_value){	// if within 90% to end 
		conveyor.set_zero_position(conveyor.get_position() - binding_value);
	}
}

// detects if a ring has been taken by intake, should only when conveyor is at resting position
bool detect_ring(){
    return conveyor_optical.get_proximity() > 100;   //  might need to change
}

// #====# sensor based movements #====#

// possible improvement: after detecting hook, move backwards slowly till hook is undetected in case of inertia overshooting 
/// IMPT: THIS USES THE PROXIMITY SENSOR TO DETECT THE CONVEYOR HOOK AND **WILL NOT WORK WHEN A RING IS BEING TRANSPORTED**
void conveyor_go_home_by_sensor(int voltage){       
	conveyor.move(voltage);
	while(conveyor_optical.get_proximity() > CONVEYOR_THRES_PROX);       // if detect hook, leave first
	while(conveyor_optical.get_proximity() < CONVEYOR_THRES_PROX);       // run till detect hook again, home is considered the point where the hook is first detected
	conveyor.brake(); 
	conveyor.tare_position();
}


// #====# sensor-less based movements #====#

double conveyor_loop_period = 883.2;        // TODO: identify and set as constant, or keep as variable and have it constantly updated
double _calibrate_at_voltage(int voltage){
	conveyor_go_home_by_sensor(voltage);
	double ori_pos = conveyor.get_position();   // identify overshoot, if any
	pros::delay(500);
	pros::lcd::print(2,"ori_pos: %f", ori_pos);
	pros::delay(5);

	// find motor distance to next hook
	conveyor.move(voltage);
	while(conveyor_optical.get_proximity() > CONVEYOR_THRES_PROX);
	while(conveyor_optical.get_proximity() < CONVEYOR_THRES_PROX);
	conveyor.brake();
	double stepped_pos = conveyor.get_position();
	pros::delay(500);
	pros::lcd::print(3,"stepped_pos: %f", stepped_pos);
	pros::delay(5);

	return stepped_pos - ori_pos;
}
// Uses the conveyor_loop_period variable and therefore requires calibration before using 
// code is also blocking (for now?)
void conveyor_go_home(int voltage, double conveyor_loop_dist){     
    bound_conveyor_position(conveyor_loop_period);
	conveyor.move(voltage);
	while(conveyor.get_position() < conveyor_loop_dist);
    conveyor.brake();
}
bool conveyor_go_to_absolute(double percentage_position, int voltage){
    bound_conveyor_position(conveyor_loop_period);
    double as_encoder_pos = percentage_position * conveyor_loop_period;
    double conveyor_goal = ((conveyor.get_position() > as_encoder_pos) ? conveyor_loop_period : 0) + as_encoder_pos;
    
    pros::lcd::print(4, "goal: %.2f, cur: %.2f", conveyor_goal, conveyor.get_position());
    pros::delay(3);

	conveyor.move(voltage);
    bool conveyor_timed_out = false;
    uint32_t start_time = pros::millis();
	while(conveyor.get_position() < conveyor_goal){
        pros::lcd::print(4, "goal: %.0f, cur: %.0f t: %d", conveyor_goal, conveyor.get_position(), 1500 - pros::millis() + start_time);
        pros::delay(4);
        if(pros::millis() - start_time > 1500){
            conveyor_timed_out = true;
            break;
        }
    }
    if(conveyor_timed_out){
        pros::lcd::print(7, "fuck");
        conveyor.move(-70);
        pros::delay(100);
    }else{
        pros::lcd::print(7,"okay");
    }
    conveyor.brake();
    return conveyor_timed_out;
}

void conveyor_go_to(int conveyor_step);  // prototype
/* conveyor_step variable - state machine's state / position
 0 - resting (right below the intersection of intake and conveyor)
 1 - store (middle of conveyor, possible to store another ring at intersection)
 2 - score (score the ring, returns to resting position)
*/
int conveyor_step = 0;
void step_conveyor(){
    conveyor_step = (conveyor_step+1) % 4;
    pros::lcd::print(1, "stp: %d/3, pos: %f", conveyor_step, conveyor.get_position());
    conveyor_go_to(conveyor_step);
}

bool detected_ring_before = false;

void conveyor_go_to(int input_conveyor_step){
    switch(input_conveyor_step){
        case 0:
            conveyor_step = 0;
            conveyor_go_to_absolute(0.55, 40);       // go to rest     (allowed to receive)
            break;
        case 1:
            conveyor_step = 1;
            conveyor_go_to_absolute(0.35, 40);       // store       (waiting to score)
            // check colour
            //conveyor_optical.
            break;
        case 2:
        {
            conveyor_step = 2;
            detected_ring_before = false;                                   // score if ours*
            bool should_continue = false;
            if (is_ring_ours||ignore_colour)
                should_continue = !conveyor_go_to_absolute(0.90, 110);     // Score     (score, now rehome)
            else 
                should_continue = !conveyor_go_to_absolute(0.67, 127);
                
                conveyor.move(-70);
                pros::delay(100);
                conveyor.brake();
            is_ring_ours = false;
            pros::delay(500);           // TODO: make async
            if (should_continue) step_conveyor();
            break;
        }
        case 3:
            conveyor_step = 3;
            conveyor_go_home_by_sensor(40);     // rehome   (rehomed, now wait to receive)
            pros::delay(500);           // TODO: make async
            step_conveyor();
            break;
    }
}

void calibrate_conveyor(){
    double conveyor_loop_periods_sense = _calibrate_at_voltage(40);

    conveyor_go_home(40,conveyor_loop_periods_sense);
    pros::delay(100);
    
    double conveyor_loop_period_senseless = conveyor.get_position();

    if(conveyor_optical.get_proximity() > CONVEYOR_THRES_PROX){
        pros::lcd::print(6, "Conveyor passed %d", conveyor_optical.get_proximity());
        conveyor_loop_period = conveyor_loop_periods_sense;
    }else{
        pros::lcd::print(6, "Conveyor failed %d", conveyor_optical.get_proximity());
        // oh no, the motors might have slipped too fast 
    }
    pros::lcd::print(0, "C_S: %f, C_SL: %f", conveyor_loop_periods_sense, conveyor_loop_period_senseless);
    pros::delay(5);
}

void check_for_ring(){
    if (!detected_ring_before && detect_ring()){
        // advance conveyor to "store"
        conveyor_go_to(1);
        detected_ring_before = true;
        detected_ring_time = 100;       // this can be reformatted into a task
    }
}

bool same_colour(){
    pros::c::optical_rgb_s_t detected_colour = conveyor_optical.get_rgb();
    //red ring: 360, 167, 135  /  blue ring: 120, 178, 256
    double r = detected_colour.red, g = detected_colour.green, b = detected_colour.blue;
    bool is_valid = fabs(b-r) > 50;        // needs quite a big difference in color to count
    pros::lcd::print(7, "c: %.1f/%.1f/%.1f: %s", r, g, b, !is_valid ? "not sure" : ((b > r) ? "blue" : "red"));
    if(!is_valid) return false;
    return ((b > r) == is_we_blue_alliance);        // check if its blue and compare if we are blue

}