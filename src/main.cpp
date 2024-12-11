#include "main.h"

void disabled(){}
void competition_initialize(){}

//Function to determine sign of a integer variable, returns bool
template <typename T> int sgn(T val){
    return (T(0) < val) - (val < T(0));
}




void serialRead(void* params){
    vexGenericSerialEnable(SERIALPORT - 1, 0);
    vexGenericSerialBaudrate(SERIALPORT - 1, 115200);
    pros::delay(10);
    pros::screen::set_pen(COLOR_BLUE);
    double distX, distY = 0;
    while(true){
        uint8_t buffer[256];
        int bufLength = 256;
        int32_t nRead = vexGenericSerialReceive(SERIALPORT - 1, buffer, bufLength);
        if(nRead >= 0){
            std::stringstream dataStream("");
            bool recordOpticalX, recordOpticalY = false;
            for(int i = 0;i < nRead; i++){
                char thisDigit = (char)buffer[i];
                if(thisDigit == 'D' || thisDigit == 'I' || thisDigit == 'A' || thisDigit == 'X'||  thisDigit == 'C'||  thisDigit == 'Y'){
                    recordOpticalX = false;
                    recordOpticalY = false;
                }
                if(thisDigit == 'C'){
                    recordOpticalX = false;
                    dataStream >> distX;
                    pros::lcd::print(1, "Optical Flow:");
                    pros::lcd::print(2, "distX: %.2lf", distX*10);
                    dataStream.str(std::string());
                }
                if(thisDigit == 'D'){
                    recordOpticalY = false;
                    dataStream >> distY;
                    global_distY = distY/100;
                    pros::lcd::print(3, "distY: %.2lf", distY*10);
                    dataStream.str(std::string());
                }
                if (recordOpticalX) dataStream << (char)buffer[i];
                if (recordOpticalY) dataStream << (char)buffer[i];
                if (thisDigit == 'X') recordOpticalX = true;
                if (thisDigit == 'Y') recordOpticalY = true;
            }
        }
        pros::Task::delay(25);
    }
}

void brake(){ //brakes all base motors
    luA.brake();
    ruA.brake();
    luB.brake();
    ruB.brake();
    llA.brake();
    rlA.brake();
    llB.brake();
    rlB.brake();
    pros::delay(1);
}

void tareBaseMotorEncoderPositions() //tares all base motor encoder positions
{
    luA.tare_position();
    ruA.tare_position();
    luB.tare_position();
    ruB.tare_position();
    llA.tare_position();
    rlA.tare_position();
    llB.tare_position();
    rlB.tare_position();
    pros::delay(1);
}

void clampVoltage(double lu, double ll, double ru, double rl)
{
    //if any of lu, ll, ru or rl are too big, we need to scale them, and we must scale them all by the same amount so we dont throw off the proportions
    if(fabs(lu) > MAX_VOLTAGE || fabs(ll) > MAX_VOLTAGE || fabs(ru) > MAX_VOLTAGE || fabs(rl) > MAX_VOLTAGE)
    {
        //figure out which of lu, ll, ru or rl has the largest magnitude
        double max = fabs(lu);
        if(max < fabs(ll))
            max = fabs(ll);
        if(max < fabs(ru))
            max = fabs(ru);
        if(max < fabs(rl))
            max = fabs(rl);
        double VoltageScalingFactor = max / MAX_VOLTAGE; //this will definitely be positive, hence it wont change the sign of lu, ll, ru or rl.
        lu = lu / VoltageScalingFactor;
        ll = ll / VoltageScalingFactor;
        ru = ru / VoltageScalingFactor;
        rl = rl / VoltageScalingFactor;
    }
}
double wrapAngle(double angle){ //forces the angle to be within the -180 < angle < 180 range
    if (angle > 180.0)
        while (angle > 180.0)
            angle -= 360.0;
    else if (angle < -180.0)
        while (angle< -180.0)
            angle += 360.0;  
    return angle;
}

double getNormalizedSensorAngle(pros::Rotation &sensor){ //Converts rotational sensor readings into degrees and bounds it between -180 to 180
    double angle = sensor.get_angle() / 100.0; //Convert from centidegrees to degrees
    return wrapAngle(angle); //forces the angle to be within the -180 < angle < 180 range
}

vector3D normalizeJoystick(int x_in, int y_in){ //convert translation joystick input to polar vector
    double angle = atan2(y_in, x_in) * TO_DEGREES; //angle of translation polar vector. Note that atan2 automatically considers the sign to deal with the trigonometry quadrant
    double scaleLength;
    double magnitude; //magnitude of translation polar vector
    double length = sqrt(x_in * x_in + y_in * y_in);
    vector3D out;
    if(length < DEADBAND){ //if the joystick is too close to the origin, dont bother moving (this is to correct for stick drift, where the joystick doesnt default to the 0,0 position due to physical damage)
        out.load(0.0, 0.0, 0.0); //assign zero values to the xyz attributes of the vector3D named "out"
        return out;
    }
    //forcing the joystick output to be a circle not the square bounding box of the joystick
    //for any radial line of the circle, we find its length from the deadband radius to the radius of the circle of the joystick, then map the speed from 0 to 1 of that length
    //this means that in any direction, we can reach our full range of speeds without being limited
    //use CSC or SEC as required
    if((angle > 45.0  && angle < 135.0) || (angle < -45.0 && angle > -135.0))
        scaleLength = 127.0 / sin(angle * TO_RADIANS);
    else
        scaleLength = 127.0 / cos(angle * TO_RADIANS);
    
    scaleLength = fabs(scaleLength) - DEADBAND; //force scaleLength to be positive and subtract deadband
    magnitude = (length - DEADBAND) / scaleLength; //find magnitude of translation vector and scale it down (note that magnitude will always be positive)
    
    out.load(magnitude * cos(angle * TO_RADIANS), magnitude * sin(angle * TO_RADIANS), 0.0); //assign values to the xyz attributes of the vector3D named "out"
    return out;
}

vector3D normalizeRotation(int x_in){ //get rotation speed from rotation joystick
    vector3D out;
    double scaleLength = 127.0 - DEADBAND;
    if(abs(x_in) < DEADBAND){ //if the joystick is too close to the origin, dont bother moving (this is to correct for stick drift, where the joystick doesnt default to the 0,0 position due to physical damage)
        out.load(0.0, 0.0, 0.0); //assign values to the xyz attributes of the vector3D named "out"
        return out;
    }
    double value = (abs(x_in) - DEADBAND) / scaleLength; //find magnitude of rotation and scale it down
    if(x_in < 0){
        value = value * -1.0;
    }
    //this is SUPPOSED to be a vector, its not wrong
    //both normaliseRotation and normaliseJoystick return a vector for standardisation. This is intended behaviour.
    out.load(0.0, 0.0, value); //assign values to the xyz attributes of the vector3D named "out"
    return -out;
}


double angle(vector3D v1, vector3D v2){
    double dot = v1 * v2;
    double det = v1.x * v2.y - v1.y * v2.x;
    return -atan2(det, dot); //atan2 automatically considers the sign to deal with the trigonometry quadrant
}

double max(double a, double b) { //returns the larger of two doubles
    return (a > b)? a : b;
}

double min(double a, double b) { //returns the smaller of two doubles
    return (a < b)? a : b;
}

void moveBase(){
    double v_right_velocity; // target velocityy
    double v_left_velocity;


    double left_angle;
    double right_angle;
    double left_target_angle;
    double right_target_angle;
    vector3D rotational_v_vector; // vector expression for angular velocity -- only has z component
    
    vector3D current_left_vector; // current angle and the speed of the left and right wheels
    vector3D current_right_vector;

    double l_error = 0.0; // how far the left and right angles wrt to the their respective target angles
    double r_error = 0.0;

    double current_l_velocity = 0.0; // current left and right velocities of the indiv wheels
    double current_r_velocity = 0.0;
    
    double current_l_tl_error = 0.0; // speed error of the wheels wrt to target speed
    double current_r_tl_error = 0.0;

    double l_angle_pid = 0.0; // power diff for angular velocity
    double r_angle_pid = 0.0;

    double l_velocity_pid = 0.0; // power diff to reach velocity target
    double r_velocity_pid = 0.0;

    double lscale = 0; // power scale to adjust the power to the wheels i.e. less power for the wheels if position of the wheels is wrong and vv
    double rscale = 0;

    double current_angular = 0.0; // current angular velocity

    vector3D current_tl_velocity(0,0,0);

    vector3D prev_target_v(0,0,0); 
    vector3D prev_target_r(0,0,0);

    vector3D v_fterm(0,0,0); // fterm: added constants to make PID more responsive
    vector3D r_fterm(0,0,0); // based on rate of change of input from joystick

    double average_x_v = 0;
    double average_y_v = 0;

    uint64_t micros_now = -1;
    
    uint64_t micros_prev = pros::micros();
    uint64_t dt = -1;

    int32_t lu; // left upper
    int32_t ll; // left lower
    int32_t ru; // right upper
    int32_t rl; // right lower

    PID left_angle_PID(angle_kP, angle_kI, angle_kD);
    PID right_angle_PID(angle_kP, angle_kI, angle_kD);
    PID left_velocity_PID(velocity_kP, velocity_kI, velocity_kD);
    PID right_velocity_PID(velocity_kP, velocity_kI, velocity_kD);
    
    vector3D L2I_pos(WHEEL_BASE_RADIUS,0.0,0.0);

    while(true){
        left_angle = wrapAngle(getNormalizedSensorAngle(left_rotation_sensor)-90.0)*TO_RADIANS;
        right_angle = wrapAngle(getNormalizedSensorAngle(right_rotation_sensor)-90.0)*TO_RADIANS;
        current_left_vector = vector3D(cos(left_angle), sin(left_angle), 0.0);
        current_right_vector = vector3D(cos(right_angle), sin(right_angle), 0.0);

        current_l_velocity = ((luA.get_actual_velocity()+luB.get_actual_velocity()+llA.get_actual_velocity()+llB.get_actual_velocity())/4.0);
        current_r_velocity = ((ruA.get_actual_velocity()+ruB.get_actual_velocity()+rlA.get_actual_velocity()+rlB.get_actual_velocity())/4.0);

        current_angular = (current_l_velocity*sin(left_angle)+current_r_velocity*sin(right_angle))/(2.0*WHEEL_BASE_RADIUS); // current angular velocity
        average_x_v = ((current_l_velocity*cos(left_angle))+(current_r_velocity*cos(right_angle)))/2.0;
        average_y_v = ((current_l_velocity*sin(left_angle))+(current_r_velocity*sin(right_angle)))/2.0;
        current_tl_velocity.load(average_x_v,average_y_v,0.0);

        prev_target_v = target_v; // prev target velocity
        prev_target_r = target_r; // prev target rotation
        
        // TODO: switch PID to go for target angle, switch actual to use current sensor angle
        target_v = normalizeJoystick(-leftX, leftY).scalar(MAX_SPEED); // target velocity
        target_r = normalizeRotation(-rightX).scalar(MAX_ANGULAR); // target rotation

        // pros::lcd::print(3, "target_r X %%.1lf", target_r.x);
        // pros::lcd::print(4, "target_r Y %.1lf", target_r.y);
        // pros::lcd::print(5, "target_r Z %.1lf", target_r.z);

        micros_prev = micros_now;
        micros_now = pros::micros();
        dt = micros_now-micros_prev;
        v_fterm = (target_v-prev_target_v)*(v_kF/dt); // rate of change of joystick input * constant v_kF
        r_fterm = (target_r-prev_target_r)*(r_kF/dt); // rate of change of joystick input * constant r_kF
        target_v = target_v + v_fterm; // update the target_v and target_r
        target_r = target_r + r_fterm;
        
        // pros::lcd::print(6, "rot_v_y %3.8f", rotational_v_vector.y);
        // pros::lcd::print(7, "rot_v_x %3.8f", rotational_v_vector.x);
        

        rotational_v_vector = L2I_pos^target_r;
        
        v_left = target_v-rotational_v_vector;
        v_right = target_v+rotational_v_vector;

        bool reverse_right = false;
        bool reverse_left = false;
        
        // check if the angle is obtuse
        if (v_left * current_left_vector < 0){  
            // reverse if angle is obtuse for shorter rotation
            v_left = -v_left;
            reverse_left = true;
        }

        if (v_right * current_right_vector < 0){  
            // reverse if angle is obtuse for shorter rotation
            v_right = -v_right;
            reverse_right = true;
        }

        v_right_velocity = SPEED_TO_RPM* TRANSLATE_RATIO*(v_right*current_right_vector);
        v_left_velocity = SPEED_TO_RPM* TRANSLATE_RATIO*(v_left*current_left_vector);

        if(reverse_left){
            v_left_velocity = -v_left_velocity;
        }

        if(reverse_right){
            v_right_velocity = -v_right_velocity;
        }

        // calculate the error angle
        l_error = angle(current_left_vector, v_left);
        r_error = angle(current_right_vector, v_right);
        if (std::isnan(l_error) || std::isnan(r_error)) {
            l_error = 0.0; r_error = 0.0;
        }

        //calculate the wheel error
        current_l_tl_error = (v_left_velocity-current_l_velocity);
        current_r_tl_error = (v_right_velocity-current_r_velocity);

        // velocity pid: based on the rate of change of velocity, pid updates the power the wheels
        l_velocity_pid += left_velocity_PID.step(current_l_tl_error);
        r_velocity_pid += right_velocity_PID.step(current_r_tl_error);

        // angle pid: based on error, pid updates the power to the wheels
        l_angle_pid = left_angle_PID.step(l_error);
        r_angle_pid = right_angle_PID.step(r_error);

        // higher base_v: drifts and lower base_v: lags
        lscale = scale * ((1.0-base_v)*fabs((l_error))+base_v);
        rscale = scale * ((1.0-base_v)*fabs((r_error))+base_v);

        lu = (int32_t)std::clamp(lscale * (l_velocity_pid + l_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE); //this side seems less powerful on the robot
        ll = (int32_t)std::clamp(lscale * (l_velocity_pid - l_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE);
        ru = (int32_t)std::clamp(rscale * (r_velocity_pid + r_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE);
        rl = (int32_t)std::clamp(rscale * (r_velocity_pid - r_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE);
        

        // pros::lcd::print(6, "ru %3.8d", ru);
        // pros::lcd::print(7, "rl %3.8d", rl);
        // pros::lcd::print(0, "target_r %3.83", target_r);
        // pros::lcd::print(0, "la %3.3f", left_angle);
        // pros::lcd::print(1, "lu %3.3d", lu);
        // pros::lcd::print(2, "ll %3.3d", ll);
        
        target_r.getX();
        target_r.getY();
        target_r.getX();
        // pros::lcd::print(3, "ra %3.3f", right_angle);

        luA.move_voltage(lu);
        luB.move_voltage(lu);

        llA.move_voltage(ll);
        llB.move_voltage(ll);

        ruA.move_voltage(ru);
        ruB.move_voltage(ru);

        rlA.move_voltage(rl);
        rlB.move_voltage(rl);
    
        pros::delay(2);
    }
}



void autonomous(){
    // move_auton();
}

void initialize(){
    pros::lcd::initialize();
    
    luA.set_brake_mode(MOTOR_BRAKE_COAST);
    luB.set_brake_mode(MOTOR_BRAKE_COAST);
    llA.set_brake_mode(MOTOR_BRAKE_COAST);
    llB.set_brake_mode(MOTOR_BRAKE_COAST);
    ruA.set_brake_mode(MOTOR_BRAKE_COAST);
    ruB.set_brake_mode(MOTOR_BRAKE_COAST);
    rlA.set_brake_mode(MOTOR_BRAKE_COAST);
    rlB.set_brake_mode(MOTOR_BRAKE_COAST);
    // liftL.set_brake_mode(MOTOR_BRAKE_HOLD);
    // liftR.set_brake_mode(MOTOR_BRAKE_HOLD);

    //while(!left_rotation_sensor.reset());
    //while(!right_rotation_sensor.reset());

    left_rotation_sensor.set_data_rate(5);
    right_rotation_sensor.set_data_rate(5);

    left_rotation_sensor.set_position(0);
    right_rotation_sensor.set_position(0);

    conveyor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE); 
    roller.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE); 

    pros::Task move_base(moveBase);
    pros::Task serial_read(serialRead);

    master.clear();
}

void opcontrol(){
    while(true){
        leftX = master.get_analog(ANALOG_LEFT_X);
        leftY = master.get_analog(ANALOG_LEFT_Y);
        rightX = master.get_analog(ANALOG_RIGHT_X);
    // moveBase();

    // R1 FORWARD, R2 BACKWARD FOR CONVEYOR
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { 
		// pros::lcd::print(0, "R1 pressed, CONVEYOR FORWARD\n");
		conveyor.move(110); 

    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { 
		// pros::lcd::print(0, "R2 pressed, CONVEYOR BACKWARD\n");
		conveyor.move(-110);

    } else { 
		// pros::lcd::print(0, "CONVEYOR STOPPED\n");
      	conveyor.move(0);
    } 

	// L1 FORWARD, L2 BACKWARD FOR ROLLER (missing hardware)
	// when L1 is pressed, rollers move forward with NEGATIVE velocity??
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { 
		// pros::lcd::print(0, "L2: ROLLER backward, +ve velocity??\n");
		roller.move(110); 

    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { 
		// pros::lcd::print(0, "L1: ROLLER forward, -ve velocity??\n");
		roller.move(-110);

    } else {
		// pros::lcd::print(0, "ROLLER STOPPED\n");
		roller.move(0); 
    }

    if(master.get_digital_new_press(DIGITAL_A)) mobile_goal_actuated = !mobile_goal_actuated;
    if(mobile_goal_actuated) solenoid.set_value(1);
    else solenoid.set_value(0);

    if(master.get_digital_new_press(DIGITAL_Y)) front_roller_actuated = !front_roller_actuated;
    if(front_roller_actuated) front_roller.set_value(1);
    else front_roller.set_value(0);
	
	   pros::delay(100); 
    }
}