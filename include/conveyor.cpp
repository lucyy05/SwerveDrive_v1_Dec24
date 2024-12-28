#include "definitions.h"
#include "api.h"
//#define DISABLE_CONVEYOR_LCD_PRINTS

bool override_ignore_colour = true;
bool is_we_red_alliance = false;            // are we red and should therefore score red
bool is_ring_ours = false;                  // is the ring the same colour as ours (updates after intake, storage and like 500ms)
bool auto_trim_with_green = true;

/* TODO:
 - [ ] make multiple intake working
   - [ ] trim automatically instead of going home to trim
   - [/] trim with green instead
 - [/] yeet reliably
 - [ ] move conveyor backwards accurately
*/

/* CONVEYOR FUNCTION USAGE IN AUTON
 - run conveyor_go_to_step(3) as init to home, good calibration value has been supplied
 - place check_for_ring() in intake roller code, it'll automatically store any rings taken in and detected
 - step_conveyor() is the function to use to automatically go thru the subroutine to:
   - receive ring (from intake)
   - store (free up intake)
   - score (afterwards, immediately goes to "rehome" (if possible))
   - rehome (afterwards, immediately goes to "receive" (if possible))
   #=== good to know probably wont use ===#
 - use conveyor_go_to_absolute() to move conveyor to where you want. (you dont need to use this, use step_conveyor())
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
 - [~] fix bugs
 - [~] colour reliability
*/

// #====# conveyor util functions

int same_colour(){      // maybe like x0.7 the red

    pros::c::optical_rgb_s_t detected_colour = conveyor_optical.get_rgb();
    
    //red ring: 360, 167, 135  /  blue ring: 120, 178, 256
    double r = detected_colour.red * 0.7, g = detected_colour.green, b = detected_colour.blue;
    bool is_valid = fabs(b-r) > 50;        // needs quite a big difference in color to count

    #ifndef DISABLE_CONVEYOR_LCD_PRINTS
    pros::lcd::print(6, "c: %.1f/%.1f/%.1f: %s", r, g, b, !is_valid ? "not sure" : ((b > r) ? "blue" : "red"));
    #endif

    if(!is_valid) return -1;
    return ((b > r) ? 1 : 0);        // check if its blue and compare if we are blue
}

void bound_conveyor_position(double binding_value){
    if(conveyor.get_position() >= binding_value){	// if within 90% to end 
		conveyor.set_zero_position(conveyor.get_position() - binding_value);
	}
}

double bound_conveyor_location(double conveyor_location, double binding_value, bool reverse = false){
    if(!reverse && conveyor_location >= binding_value){
        return conveyor_location + binding_value;
    }
    if(reverse && conveyor_location <= binding_value){
        return conveyor_location - binding_value;
    }
}

// detects if a ring has been taken by intake, should only when conveyor is at resting position
bool detect_ring(){
    return conveyor_optical.get_proximity() > 100;   //  might need to change
}

// #====# sensor based movements #====#

// possible improvement: after detecting hook, move backwards slowly till hook is undetected in case of inertia overshooting 
/// IMPT: THIS USES THE PROXIMITY SENSOR TO DETECT THE CONVEYOR HOOK AND **WILL NOT WORK WHEN A RING IS BEING TRANSPORTED**
bool conveyor_go_home_by_sensor(int voltage){       
	conveyor.move(voltage);
    uint32_t start_time = pros::millis();
    bool timed_out = false;
	while(conveyor_optical.get_proximity() > CONVEYOR_PROX_THRES && !timed_out){
        pros::delay(1);
        if(pros::millis() > start_time + 2000){
            timed_out = true;
            break;
        }
    };       // if detect hook, leave first
	while(conveyor_optical.get_proximity() < CONVEYOR_PROX_THRES && !timed_out){
        pros::delay(1);
        if(pros::millis() > start_time + 2000){
            timed_out = true;
            break;
        }
    };       // run till detect hook again, home is considered the point where the hook is first detected
	conveyor.brake(); 
	conveyor.tare_position();
    if(timed_out){
        #ifndef DISABLE_CONVEYOR_LCD_PRINTS
        pros::lcd::print(7, "obstacle in the way");
        #endif
        conveyor.move(-70);
        pros::delay(100);
    }
    return timed_out;
}

bool conveyor_go_home_by_sensor_green(int voltage){       
	conveyor.move(voltage);
	conveyor.move(voltage);
    uint32_t start_time = pros::millis();
    bool timed_out = false;
	while(conveyor_optical.get_rgb().green > CONVEYOR_GREEN_THRES && !timed_out){
        //pros::lcd::print(3, "g: %.2f", conveyor_optical.get_rgb().green);
        pros::delay(1);
        if(pros::millis() > start_time + 2000){
            timed_out = true;
            break;
        }
    };       // if detect hook, leave first
	while(conveyor_optical.get_rgb().green < CONVEYOR_GREEN_THRES && !timed_out){
        pros::delay(1);
        if(pros::millis() > start_time + 2000){
            timed_out = true;
            break;
        }
    };       // run till detect hook again, home is considered the point where the hook is first detected
	conveyor.brake(); 
	conveyor.tare_position();
    if(timed_out){
        #ifndef DISABLE_CONVEYOR_LCD_PRINTS
        pros::lcd::print(7, "obstacle in the way");
        #endif
        conveyor.move(-70);
        pros::delay(100);
    }
    return timed_out;
}

// #====# sensor-less based movements #====#

double conveyor_loop_period = 854.4;
double _calibrate_at_voltage(int voltage){
	conveyor_go_home_by_sensor(voltage);
	double ori_pos = conveyor.get_position();   // identify overshoot, if any
	pros::delay(500);
    #ifndef DISABLE_CONVEYOR_LCD_PRINTS
	pros::lcd::print(2,"ori_pos: %f", ori_pos);
    #endif
	pros::delay(5);

	// find motor distance to next hook
	conveyor.move(voltage);
	while(conveyor_optical.get_proximity() > CONVEYOR_PROX_THRES);
	while(conveyor_optical.get_proximity() < CONVEYOR_PROX_THRES);
	conveyor.brake();
	double stepped_pos = conveyor.get_position();
	pros::delay(500);
    #ifndef DISABLE_CONVEYOR_LCD_PRINTS
	pros::lcd::print(3,"stepped_pos: %f", stepped_pos);
    #endif
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

int times_moved = 0;
bool conveyor_go_to_absolute(double percentage_position, int voltage){
    times_moved++;
    bound_conveyor_position(conveyor_loop_period);
    double as_encoder_pos = percentage_position * conveyor_loop_period;
    double conveyor_goal = ((conveyor.get_position() > as_encoder_pos) ? conveyor_loop_period : 0) + as_encoder_pos;

    #ifndef DISABLE_CONVEYOR_LCD_PRINTS
    pros::lcd::print(4, "goal: %.2f, cur: %.2f", conveyor_goal, conveyor.get_position());
    #endif
    pros::delay(3);

	conveyor.move(voltage);
    bool conveyor_timed_out = false;
    uint32_t start_time = pros::millis();
	while(conveyor.get_position() < conveyor_goal){
        #ifndef DISABLE_CONVEYOR_LCD_PRINTS
        pros::lcd::print(4, "goal: %.0f, cur: %.0f t: %d", conveyor_goal, conveyor.get_position(), 1500 - pros::millis() + start_time);
        #endif
        pros::delay(4);
        if(pros::millis() - start_time > 1500){
            conveyor_timed_out = true;
            break;
        }
    }
    if(conveyor_timed_out){
        #ifndef DISABLE_CONVEYOR_LCD_PRINTS
        pros::lcd::print(7, "obstacle in the way");
        #endif
        conveyor.move(-70);
        pros::delay(100);
    }else{
        #ifndef DISABLE_CONVEYOR_LCD_PRINTS
        pros::lcd::print(7, "okay");
        #endif
    }
    conveyor.brake();
    return conveyor_timed_out;
}

void conveyor_go_to_step(int input_conveyor_step, bool ignore_colour = false);  // prototype
/* conveyor_step variable - state machine's state / position
 0 - resting (right below the intersection of intake and conveyor)
 1 - store (middle of conveyor, possible to store another ring at intersection)
 2 - score (score the ring, returns to resting position)
 3 - rehome (rehomes the conveyor for better accuracy)
*/
int conveyor_step = 0;
/// @brief steps thru the conveyor stages automatically, see conveyor_step brief or conveyor_go_to_step() brief for more info
void step_conveyor(){
    conveyor_step = (conveyor_step+1) % 4;
    #ifndef DISABLE_CONVEYOR_LCD_PRINTS
    pros::lcd::print(1, "stp: %d/3, pos: %f", conveyor_step, conveyor.get_position());
    #endif
    conveyor_go_to_step(conveyor_step);
}

bool detected_ring_before = false;

bool conveyor_go_to_rest(){
    return conveyor_go_to_absolute(0.55, 50);
}
bool conveyor_go_to_store(){
    return conveyor_go_to_absolute(0.48, 50);
}
bool conveyor_go_to_score(){
    return conveyor_go_to_absolute(0.85, 120);
}
bool conveyor_go_to_yeet(){
    bool ret = conveyor_go_to_absolute(0.66, 125);      // launch out
    conveyor.move(-85);
    pros::delay(60);
    conveyor.brake();
    return ret;
}
// get to 7cm reliably
bool conveyor_score_aliance(){
    conveyor.move(-70);
    pros::delay(180);
    conveyor.brake();
    pros::delay(30);
    return conveyor_go_to_score();
}

/**
	 * Used to move the conveyor to stage of the scoring process: rest, store, score, (home)
	 * 
     * Stages:
     * 
     * 0: [rest] (ready to take in ring, when intake detects a ring, automatically transitions to [store] (behavior from check_for_ring())
     * 
     * 1: [store] (stores the ring roughly midway to allow another ring to be kept at intake
     * 
     * 2: [score] (scores(or rejects, see ignore colour) the ring, automatically goes to [home] if possible
     * 
     * 3: [home] (homes and trims the location for accuracy, automatically goes to [rest]))
	 * 
	 * \param  input_conveyor_step
	 * 				 conveyor stage to go to
     * \param  ignore_colour default: false
     *               When [score]ing, determines if it should not take colour into account
     *               When false, and ring does not match our colour (determined by is_we_blue_alliance), the rejection sequence is ran that will **not** score the ring. This behaviour is disabled when ignore_colour = true.
	 */
void conveyor_go_to_step(int input_conveyor_step, bool ignore_colour){
    switch(input_conveyor_step){
        case 0:         // go to rest     (allowed to receive)
            conveyor_step = 0;
            conveyor_go_to_rest();
            break;
        case 1:         // store       (waiting to score)
            conveyor_step = 1;
            conveyor_go_to_store();
            break;
        case 2:         // score (or reject)
        {
            conveyor_step = 2;
            bool should_continue = false;
            if (is_ring_ours||ignore_colour)
                should_continue = !conveyor_go_to_score();     // Score     (score, now rehome)
            else {
                should_continue = !conveyor_go_to_yeet();      // remove
            }
            // reset variables
            is_ring_ours = false;
            detected_ring_before = false;
            pros::delay(500);           // wait for conveyor to actually stop, can probably actually decrease this to like 50
            if (should_continue) step_conveyor();
            break;
        }
        case 3:
            {
                conveyor_step = 3;
                bool should_continue = conveyor_go_home_by_sensor_green(30);     // rehome   (rehomed, now wait to receive)
                pros::delay(500);           // can probably decrease this to like 50
                if (should_continue) step_conveyor();
                break;
            }
    }
}

/// @brief SHOULD NOT BE USED ANYMORE. Calibration number has been found (conveyor_loop_period = 854.4) this function doesnt need to be used anymore
/// Finds the calibration value for the motor distance between conveyor hooks.
void calibrate_conveyor(){
    double conveyor_loop_periods_sense = _calibrate_at_voltage(30);

    conveyor_go_home(40,conveyor_loop_periods_sense);
    pros::delay(100);
    
    double conveyor_loop_period_senseless = conveyor.get_position();

    if(conveyor_optical.get_proximity() > CONVEYOR_PROX_THRES){
        #ifndef DISABLE_CONVEYOR_LCD_PRINTS
        pros::lcd::print(6, "Conveyor passed %d", conveyor_optical.get_proximity());
        #endif
        conveyor_loop_period = conveyor_loop_periods_sense;
    }else{
        #ifndef DISABLE_CONVEYOR_LCD_PRINTS
        pros::lcd::print(6, "Conveyor failed %d", conveyor_optical.get_proximity());
        #endif
        pros::delay(100);
        calibrate_conveyor();   // try again
        // oh no, the motors might have slipped too fast 
    }
    #ifndef DISABLE_CONVEYOR_LCD_PRINTS
    pros::lcd::print(0, "C_S: %f, C_SL: %f", conveyor_loop_periods_sense, conveyor_loop_period_senseless);
    #endif
    pros::delay(5);
}

int align_ring_for_colour(){
    int colourResult = -1;
    conveyor_go_to_step(1);
    conveyor.move(40);
    while(colourResult == -1){      // while not sure of colour
        colourResult = same_colour();
    }
    conveyor.brake();
    return colourResult;
}

/// @brief Place in the main loop code for ring intake, should be run repeatedly when trying to pickup a ring.
/// Will automatically store any detected rings and take its colour reading
void check_for_ring(){
    bool detected_ring = detect_ring();
    #ifndef DISABLE_CONVEYOR_LCD_PRINTS
    pros::lcd::print(2,"%s, %s", (!detected_ring_before) ? "true": "false", detected_ring ? "true" : "false");
    #endif
    if (!detected_ring_before && detect_ring()){
        #ifndef DISABLE_CONVEYOR_LCD_PRINTS
        pros::lcd::print(3, "entered");
        #endif
        // advance conveyor to "store"
        //roller.brake();
        // pull ring till same_colour says its good
        int colourResult = align_ring_for_colour();
        // conveyor at "store", now check colour (ONLY IN AUTON)

        //conveyor_optical.set_led_pwm(0);
        detected_ring_before = true;
        //re-comment if needed
        // is_ring_ours = (colourResult != 1) ^ is_we_blue_alliance;
    }
}

/// The auton task method functions

// ind:0, status of ring at arm. ind:1, status of ring at [store]. ind:2, status of ring at intake ?
// -2: no ring, -1: not sure what colour, 0: not alliance colour, 1: alliance colour
bool continue_conveyor = true;
bool allowed_to_score = true;
void autoConveyor(){        // "stores" 3 ring, run as task?
    while(continue_conveyor){
        // wait for ring
        if(!detected_ring_before && detect_ring()){
            // maybe stop intake
            int colourResult = align_ring_for_colour();
            conveyor.move(0);
            detected_ring_before = true;
            //uncomment if needed
            // is_ring_ours = (colourResult != 1) ^ is_we_blue_alliance;
            #ifndef DISABLE_CONVEYOR_LCD_PRINTS
            pros::lcd::print(2,"c: %d, IRO: %s", colourResult, is_ring_ours ? "Yes" : "No");
            #endif
            pros::delay(50);
        }
        if(allowed_to_score && detected_ring_before){
            step_conveyor();
            pros::delay(50);
        }
        pros::delay(50);
    }
    continue_conveyor = true;   // lets it start again
}

/// @brief Homes the conveyor to a known location and then prepare for intake
void conveyor_init(){
    conveyor_optical.set_led_pwm(100);
    conveyor_step = 3;
    conveyor_go_to_step(3);

}