#include "main.h"

// be sure to check if 'is_we_blue_alliance' is set correctly in conveyor.cpp

void disabled(){}
void competition_initialize(){}

//Function to determine sign of a integer variable, returns bool
template <typename T> int sgn(T val){
    return (T(0) < val) - (val < T(0));
}

void setBrakeModes(){
    luA.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    luB.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    llA.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    llB.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    ruA.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    ruB.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    rlA.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    rlB.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    slam_dunk_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    conveyor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    roller.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void serialRead(void* params){
    vexGenericSerialEnable(SERIALPORT - 1, 0);
    vexGenericSerialBaudrate(SERIALPORT - 1, 115200);
    pros::delay(10);
    pros::screen::set_pen(COLOR_RED);
    double dist_X = 0.0;
    double dist_Y = 0.0;
    double prevDist_x = 0.0;
    double prevDist_y = 0.0;
    while(true){
        uint8_t buffer[256];
        int bufLength = 256;
        int32_t nRead = vexGenericSerialReceive(SERIALPORT - 1, buffer, bufLength);
        if(nRead >= 0){
            std::stringstream dataStream("");
            bool recordOpticalX = false;
            bool recordOpticalY = false;
            for(int i = 0;i < nRead; i++){
                char thisDigit = (char)buffer[i];
                if(thisDigit == 'D' || thisDigit == 'I' || thisDigit == 'A' || thisDigit == 'X'||  thisDigit == 'C'||  thisDigit == 'Y'){
                    recordOpticalX = false;
                    recordOpticalY = false;
                }
                if(thisDigit == 'C'){
                    recordOpticalX = false;
                    dataStream >> dist_X;
                    global_distX = dist_X*-10.0;
                    optical_v_x = fabs((fabs(global_distX) - fabs(prevDist_x))) / 0.002;
                    //pros::lcd::print(1, "Optical Flow:");
                    pros::lcd::print(0, "distX: %.2lf, distY: %.2lf", global_distX, global_distY);
                    dataStream.str(std::string());
                    std::stringstream dataStream("    ");
                    prevDist_x = dist_X*-10.0;
                    //pros::lcd::print(4, "optical v_x: %.2lf", optical_v_x);
                }
                if(thisDigit == 'D'){
                    recordOpticalY = false;
                    dataStream >> dist_Y;
                    global_distY = dist_Y*-10.0;
                    optical_v_y = fabs((fabs(global_distY) - fabs(prevDist_y))) / 0.002;
                    //pros::lcd::print(3, "distY: %.2lf", global_distY);
                    dataStream.str(std::string());
                    std::stringstream dataStream("    ");
                    prevDist_y = dist_Y*-10.0;
                    //pros::lcd::print(5, "optical v_y: %.2lf", optical_v_y);
                }
                if (recordOpticalX) dataStream << (char)buffer[i];
                if (recordOpticalY) dataStream << (char)buffer[i];
                if (thisDigit == 'X') recordOpticalX = true;
                if (thisDigit == 'Y') recordOpticalY = true;
            }
        }
        pros::Task::delay(2);
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

void clampVoltage() 
{
    //if any of lu, ll, ru or rl are too big, we need to scale them, and we must scale them all by the same amount so we dont throw off the proportions
    if(abs(lu) > MAX_VOLTAGE || abs(ll) > MAX_VOLTAGE || abs(ru) > MAX_VOLTAGE || abs(rl) > MAX_VOLTAGE)
    {
        //figure out which of lu, ll, ru or rl has the largest magnitude
        int32_t max = abs(lu);
        if(max < abs(ll))
            max = abs(ll);
        if(max < abs(ru))
            max = abs(ru);
        if(max < abs(rl))
            max = abs(rl);
        double VoltageScalingFactor = ((double) max) / MAX_VOLTAGE; //this will definitely be positive, hence it wont change the sign of lu, ll, ru or rl.
        lu = (int32_t)((double)lu / VoltageScalingFactor);
        ll = (int32_t)((double)ll / VoltageScalingFactor);
        ru = (int32_t)((double)ru / VoltageScalingFactor);
        rl = (int32_t)((double)rl / VoltageScalingFactor);
        // master.print(0,0,"%6d",lu);
    }
}

void move_voltage_wheels(int32_t lu, int32_t ll, int32_t ru, int32_t rl){
    luA.move_voltage(lu);
    luB.move_voltage(lu);

    llA.move_voltage(ll);
    llB.move_voltage(ll);

    ruA.move_voltage(ru);
    ruB.move_voltage(ru);

    rlA.move_voltage(rl);
    rlB.move_voltage(rl);
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

    //assign values to the xyz attributes of the vector3D named "out"
    out.load(magnitude * cos(angle * TO_RADIANS), magnitude * sin(angle * TO_RADIANS), 0.0);
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
    if(x_in > 0){
        value = value * -1.0;// Note that positive x corresponds to negative rotation speed as rotation speed is anticlockwise.
    }
    //this is SUPPOSED to be a vector, its not wrong
    //both normaliseRotation and normaliseJoystick return a vector for standardisation. This is intended behaviour.
    out.load(0.0, 0.0, value); //assign values to the xyz attributes of the vector3D named "out"
    return out;
}


double angle(vector3D v1, vector3D v2){
    double dot = v1 * v2;
    double det = v1.x * v2.y - v1.y * v2.x;
    return -1.0* atan2(det, dot); //atan2 automatically considers the sign to deal with the trigonometry quadrant
}

double max(double a, double b) { //returns the larger of two doubles
    return (a > b)? a : b;
}

double min(double a, double b) { //returns the smaller of two doubles
    return (a < b)? a : b;
}

// Driver code
void moveBase(){ 
    double v_right_velocity; // target velocity magnitude
    double v_left_velocity; 
 
    double battery_voltage;

    double left_angle; 
    double right_angle; 
    double left_target_angle; 
    double right_target_angle; 
    //vector3D rotational_v_vector; // vector expression for angular velocity -- only has z component 
     
    vector3D current_left_vector; // direction unit vector for wheels
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
 
    vector3D current_tl_velocity(0,0,0); //current transaltional velocity
 
    vector3D prev_target_v(0,0,0);  
    vector3D prev_target_r(0,0,0); 
 
    vector3D v_fterm(0,0,0); // fterm: added constants to make PID more responsive 
    vector3D r_fterm(0,0,0); // based on rate of change of input from joystick 
 
    double average_x_v = 0; 
    double average_y_v = 0; 
 
    uint64_t micros_now = -1; 
     
    uint64_t micros_prev = pros::micros(); 
    uint64_t dt = -1; 
 
    //voltages
    int32_t lu = 0; // left upper 
    int32_t ll = 0; // left lower 
    int32_t ru = 0; // right upper 
    int32_t rl = 0; // right lower 
 
    //PID instances
    PID left_angle_PID(angle_kP_left, angle_kI_left, angle_kD_left); 
    PID right_angle_PID(angle_kP_right, angle_kI_right, angle_kD_right); 
    PID left_velocity_PID(velocity_kP, velocity_kI, velocity_kD); 
    PID right_velocity_PID(velocity_kP, velocity_kI, velocity_kD); 
    PID rotate_robot_PID(azim_kP, azim_kI, azim_kD);
     
    vector3D L2I_pos(WHEEL_BASE_RADIUS,0.0,0.0); 
    vector3D imu_angular;
    vector3D angular_error;
    vector3D rot_pid;
    vector3D rot_FF;

    double rot_vector_double = 0.0;
    double rot_pid_double = 0.0;
    double gyro_rate = 0.0;
    double a_err_d = 0.0;   //angular error as a double

    bool toggle_flag = true;
    bool rot_pid_ena = true;

    while(true){ 
        target_v = normalizeJoystick(leftX, leftY).scalar(MAX_SPEED); // target velocity 
        // to be updated: leftX = 0 to remove left and right translations 
        target_r = normalizeRotation(rightX).scalar(MAX_ANGULAR*MAX_ANGULAR_SCALE); // target rotation 

        left_angle = wrapAngle(getNormalizedSensorAngle(left_rotation_sensor)-90.0)*TO_RADIANS;     //takes robot right as 0
        right_angle = wrapAngle(getNormalizedSensorAngle(right_rotation_sensor)-90.0)*TO_RADIANS;   //Y axis positive is front
        
        current_left_vector = vector3D(cos(left_angle), sin(left_angle), 0.0);  
        current_right_vector = vector3D(cos(right_angle), sin(right_angle), 0.0); 
 
        current_l_velocity = ((luA.get_actual_velocity()+luB.get_actual_velocity()+llA.get_actual_velocity()+llB.get_actual_velocity())/4.0); 
        current_r_velocity = ((ruA.get_actual_velocity()+ruB.get_actual_velocity()+rlA.get_actual_velocity()+rlB.get_actual_velocity())/4.0); 
 
        current_angular = (-current_l_velocity*sin(left_angle)+current_r_velocity*sin(right_angle))/(2.0*WHEEL_BASE_RADIUS); // current angular velocity 
        average_x_v = ((current_l_velocity*cos(left_angle))+(current_r_velocity*cos(right_angle)))/2.0; 
        average_y_v = ((current_l_velocity*sin(left_angle))+(current_r_velocity*sin(right_angle)))/2.0; 
        current_tl_velocity.load(average_x_v,average_y_v,0.0); 
 
        prev_target_v = target_v; // prev target velocity 
        prev_target_r = target_r; // prev target rotation 
        
        if(imu.is_calibrating() ){
            gyro_rate = current_angular;    // ignore gyro while calibrating, use encoder values
        }else{
            gyro_rate = -1.0 * imu.get_gyro_rate().z * TO_RADIANS;
        }

        imu_angular = vector3D(0.0,0.0, gyro_rate); // Radians per second, loaded as vector

        battery_voltage = pros::battery::get_voltage();
        if(battery_voltage>MAX_VOLTAGE){
            battery_voltage = MAX_VOLTAGE;
        }
         
        micros_prev = micros_now; 
        micros_now = pros::micros(); 
        dt = micros_now-micros_prev; 
        v_fterm = (target_v - prev_target_v)*(v_kF/dt); // rate of change of joystick input * constant v_kF 
        r_fterm = (target_r - prev_target_r)*(r_kF/dt); // rate of change of joystick input * constant r_kF 
        target_v = target_v + v_fterm; // update the target_v and target_r 
        target_r = target_r + r_fterm; 
        angular_error = target_r - imu_angular;
        
    
        if(fabs(angular_error.z) < ANGULAR_THRESH || abs(rightY)>90){//right stick up or slow rotation
            angular_error.load(0.0, 0.0, 0.0);
            rot_pid_double = 0.0;
        }
    
        
        a_err_d = angular_error.getZ();
        rot_pid_double += rotate_robot_PID.step(a_err_d);

        rot_FF = (target_r^L2I_pos).scalar(r_kF_STATIC);
        rot_vector_double = rot_pid_double + rot_FF.getY();
        rot_pid = vector3D(0.0, rot_vector_double, 0.0);
            
        v_left = target_v-rot_pid; //in order to rotate counterclockwise
        v_right = target_v + rot_pid; 
 
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
 
        v_right_velocity = SPEED_TO_RPM* TRANSLATE_RATIO*(v_right*current_right_vector);    //dot product should already
        v_left_velocity = SPEED_TO_RPM* TRANSLATE_RATIO*(v_left*current_left_vector);       //compensate angle drift?
 
        if(reverse_left){ 
            v_left_velocity = -v_left_velocity; 
        }
 
        if(reverse_right){ 
            v_right_velocity = -v_right_velocity; 
        } 
 
        // calculate the error angle 
        if(target_v.norm()>0.1 || target_r.norm()>0.01){
            l_error = angle(v_left, current_left_vector); 
            r_error = angle(v_right, current_right_vector); 
        }else{//NOT TL NOT ROTATE
            l_error = 0.0;  //DO NOT CHANGE WHEEL ANGLE IF NOT MOVING WHEEL
            r_error = 0.0;  //THIS IS FOR BETTER BRAKING PERFORMANCE
        }
        
        if (std::isnan(l_error) || std::isnan(r_error)) { 
            l_error = 0.0;
            r_error = 0.0; 
        } 
 
        //calculate the wheel error 
        current_l_tl_error = (v_left_velocity-current_l_velocity); 
        current_r_tl_error = (v_right_velocity-current_r_velocity); 

        // velocity pid: based on the rate of change of velocity, pid updates the power the wheels 
        l_velocity_pid += left_velocity_PID.step(current_l_tl_error); 
        r_velocity_pid += right_velocity_PID.step(current_r_tl_error); 
 
        // angle pid: based on error, pid updates the power to the wheels 
        l_angle_pid = left_angle_PID.step(l_error); //power to force anticlockwise aiming
        r_angle_pid = right_angle_PID.step(r_error); 
 
        // higher base_v: drifts and lower base_v: lags 
        lscale = (battery_voltage/MAX_VOLTAGE) * scale;// * ((1.0-base_v)*fabs((l_error))+base_v); 
        rscale = (battery_voltage/MAX_VOLTAGE) * scale;// * ((1.0-base_v)*fabs((r_error))+base_v); 
        
        // lu = (int32_t)std::clamp(lscale * (l_velocity_pid + l_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE); //this side seems less powerful on the robot 
        // ll = (int32_t)std::clamp(lscale * (l_velocity_pid - l_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE); 
        // ru = (int32_t)std::clamp(rscale * (r_velocity_pid + r_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE); 
        // rl = (int32_t)std::clamp(rscale * (r_velocity_pid - r_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE); 
        
        lu = (int32_t)lscale * (l_velocity_pid + l_angle_pid); 
        ll = (int32_t)lscale * (l_velocity_pid - l_angle_pid); 
        ru = (int32_t)rscale * (r_velocity_pid + r_angle_pid); 
        rl = (int32_t)rscale * (r_velocity_pid - r_angle_pid); 
        clampVoltage();
        
        move_voltage_wheels(lu,ll,ru,rl);
        pros::delay(2); 
    }
}



// Helper function to check if the motor is at the target
bool isMotorAtTarget(int port, int target) {
    int currentPosition = pros::c::motor_get_position(port);
    return (currentPosition >= target - 5) && (currentPosition <= target + 5);
}

// Arms code
// Arms control function
void slamDunk(){
    double Derivative = 0.0;
    double prevError = 0.0;
    double Error = 0.0;
    double Integral = 0.0;
    defaultSlamValue = slam_dunk.get_value();
    //master.print(3,0,"%d", defaultSlamValue);
    while(true){
        switch (slammingState){
            case SLAM_START_STATE: //resting position
                slam_target = defaultSlamValue;
                break;
            case SLAM_MID_STATE: //midpoint - holding position
                slam_target = defaultSlamValue - 145;
                break;
            case SLAM_EXTENDED_STATE: //extended all the way
                slam_target = defaultSlamValue - 1555;
                break;
            default:
                slam_target = defaultSlamValue;
                break;
        }

        Derivative = prevError - Error;
        Error = fabs(slam_target - slam_dunk.get_value());
        Integral += Error;
        double motorPower = slam_Kp * Error + slam_Kd * Derivative + slam_Ki * Integral;

        if (fabs(Error) <= 5.0) {
            slam_dunk_motor.move(0);
            slam_dunk_motor.brake();
        } else {
            if (slam_target > slam_dunk.get_value() + 10.0) {
                slam_dunk_motor.move(motorPower);
            } else if (slam_target < slam_dunk.get_value() - 10.0) {
                slam_dunk_motor.move(-motorPower);
            }
        }
        prevError = Error;
        pros::Task::delay(15);
    }
}

void moveBaseAutonomous(double targetX, double targetY){
    bool reverse = false;
    double v_right_velocity; // target velocity magnitude
    double v_left_velocity;

    double battery_voltage;

    double left_angle; 
    double right_angle; 
    double left_target_angle; 
    double right_target_angle; 
    //vector3D rotational_v_vector; // vector expression for angular velocity -- only has z component 

    vector3D current_left_vector; // direction unit vector for wheels
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
 
    vector3D current_tl_velocity(0,0,0); //current transaltional velocity
 
    vector3D prev_target_v(0,0,0);  
    vector3D prev_target_r(0,0,0); 

    vector3D v_fterm(0,0,0); // fterm: added constants to make PID more responsive 
    vector3D r_fterm(0,0,0); // based on rate of change of input from joystick 

    double average_x_v = 0; 
    double average_y_v = 0; 

    uint64_t micros_now = -1; 

    uint64_t micros_prev = pros::micros(); 
    uint64_t dt = -1; 

    //voltages
    int32_t lu = 0; // left upper 
    int32_t ll = 0; // left lower 
    int32_t ru = 0; // right upper 
    int32_t rl = 0; // right lower 

    //PID instances
    PID left_angle_PID(auton_angle_kP_left, auton_angle_kI_left, auton_angle_kD_left); 
    PID right_angle_PID(auton_angle_kP_right, auton_angle_kI_right, auton_angle_kD_right); 
    PID left_velocity_PID(auton_l_velocity_kP, auton_l_velocity_kI, auton_l_velocity_kD); 
    PID right_velocity_PID(auton_r_velocity_kP, auton_r_velocity_kI, auton_r_velocity_kD); 
    PID rotate_robot_PID(azim_kP, azim_kI, azim_kD);

    PID delta_X_PID(auton_distance_kP, auton_distance_kI, auton_distance_kD);
    PID delta_Y_PID(auton_distance_kP, auton_distance_kI, auton_distance_kD);

    vector3D L2I_pos(WHEEL_BASE_RADIUS,0.0,0.0); 
    vector3D imu_angular;
    vector3D angular_error;
    vector3D rot_pid;
    vector3D rot_FF;

    double rot_vector_double = 0.0;
    double rot_pid_double = 0.0;
    double gyro_rate = 0.0;
    double a_err_d = 0.0;   //angular error as a double

    double offsetX = fabs(global_distX);
    double offsetY = fabs(global_distY);

    double errorX = 0.0;
    double errorY = 0.0;

    double target_v_x;
    double target_v_y;

    if(targetX < 0.0 || targetY < 0.0)
        reverse = true;
    else
        reverse = false;

    // if(targetX == 0.0 && targetY > 0.0)
    //     autonDirection = NORTH;

    while(true){
        if(fabs(targetX) > 0.0)
            errorX = fabs(fabs(targetX) - fabs((fabs(global_distX) - offsetX)));
        else if(fabs(targetX) <= 0.0)
            errorX = 0.0;
        if(fabs(targetY) > 0.0)
            errorY = fabs(fabs(targetY) - fabs((fabs(global_distY) - offsetY)));
        else if(fabs(targetY) <= 0.0)
            errorY = 0.0;

        // if(fabs(errorX) <= 2.0){
        //     brake();
        //     break;
        // }

        if((fabs(errorX) <= 5.0 || fabs(errorX) > fabs(targetX)) && (fabs(errorY) <= 5.0 || fabs(errorY) > fabs(targetY))){
            brake();
            break;
        }

        target_v_x = delta_X_PID.step(errorX);
        target_v_y = delta_Y_PID.step(errorY);
        // target_v_x = errorX;
        // target_v_y = errorY;
        //target_v = vector3D(target_v_x, target_v_y, 0.0);
        if(reverse == false)
            target_v = normalizeJoystick(target_v_x, target_v_y).scalar(MAX_SPEED*0.8); // target velocity 
        else
            target_v = normalizeJoystick(-target_v_x, -target_v_y).scalar(MAX_SPEED*0.8); // target velocity 
        target_r = normalizeRotation(0.0).scalar(MAX_ANGULAR*0.8); // target rotation
        pros::lcd::print(1,"error_y: %.1lf", errorY);
        //pros::lcd::print(0,"target_v_x: %.1lf", target_v_x);
        //pros::lcd::print(1,"target_v_y: %.1lf", target_v_y);

        //pros::lcd::print(2,"error_x: %.1lf", errorX);
        //pros::lcd::print(3,"error_y: %.1lf", errorY);

        //takes robot right as 0
        //Y axis positive is front
        left_angle = wrapAngle(getNormalizedSensorAngle(left_rotation_sensor)-90.0)*TO_RADIANS;
        right_angle = wrapAngle(getNormalizedSensorAngle(right_rotation_sensor)-90.0)*TO_RADIANS;

        current_left_vector = vector3D(cos(left_angle), sin(left_angle), 0.0);  
        current_right_vector = vector3D(cos(right_angle), sin(right_angle), 0.0); 

        current_l_velocity = ((luA.get_actual_velocity()+luB.get_actual_velocity()+llA.get_actual_velocity()+llB.get_actual_velocity())/4.0); 
        current_r_velocity = ((ruA.get_actual_velocity()+ruB.get_actual_velocity()+rlA.get_actual_velocity()+rlB.get_actual_velocity())/4.0); 

        current_angular = (-current_l_velocity*sin(left_angle)+current_r_velocity*sin(right_angle))/(2.0*WHEEL_BASE_RADIUS); // current angular velocity 
        average_x_v = ((current_l_velocity*cos(left_angle))+(current_r_velocity*cos(right_angle)))/2.0; 
        average_y_v = ((current_l_velocity*sin(left_angle))+(current_r_velocity*sin(right_angle)))/2.0; 
        current_tl_velocity.load(average_x_v,average_y_v,0.0); 

        prev_target_v = target_v; // prev target velocity 
        prev_target_r = target_r; // prev target rotation 

        if(imu.is_calibrating()){
            gyro_rate = current_angular;    // ignore gyro while calibrating, use encoder values
        }else{
            gyro_rate = -1.0 * imu.get_gyro_rate().z * TO_RADIANS;
        }

        imu_angular = vector3D(0.0,0.0, gyro_rate); // Radians per second, loaded as angle

        battery_voltage = pros::battery::get_voltage();

        micros_prev = micros_now; 
        micros_now = pros::micros(); 
        dt = micros_now-micros_prev; 
        v_fterm = (target_v - prev_target_v)*(v_kF/dt); // rate of change of joystick input * constant v_kF 
        r_fterm = (target_r - prev_target_r)*(r_kF/dt); // rate of change of joystick input * constant r_kF 
        target_v = target_v + v_fterm; // update the target_v and target_r 
        target_r = target_r + r_fterm; 
            
        angular_error = target_r - imu_angular;
        if(fabs(angular_error.z) < ANGULAR_THRESH){
            angular_error.load(0.0,0.0,0.0);
            rot_pid_double = 0.0;
        }

        a_err_d = angular_error.getZ();
        rot_pid_double += rotate_robot_PID.step(a_err_d);
        rot_FF = (target_r^L2I_pos).scalar(r_kF_STATIC);
        rot_vector_double = rot_pid_double + rot_FF.getY();
        rot_pid = vector3D(0.0, rot_vector_double, 0.0);
            
        v_left = target_v-rot_pid; //in order to rotate counterclockwise
        v_right = target_v+rot_pid; 

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

        v_right_velocity = SPEED_TO_RPM* TRANSLATE_RATIO*(v_right*current_right_vector);    //dot product should already
        v_left_velocity = SPEED_TO_RPM* TRANSLATE_RATIO*(v_left*current_left_vector);       //compensate angle drift?

        if(reverse_left){
            v_left_velocity = -v_left_velocity; 
        }

        if(reverse_right){ 
            v_right_velocity = -v_right_velocity; 
        } 

        // calculate the error angle 
        l_error = angle(v_left, current_left_vector); 
        r_error = angle(v_right, current_right_vector); 
        if (std::isnan(l_error) || std::isnan(r_error)) { 
            l_error = 0.0; r_error = 0.0; 
        } 

        //calculate the wheel error 
        current_l_tl_error = (v_left_velocity-current_l_velocity); 
        current_r_tl_error = (v_right_velocity-current_r_velocity); 
        //pros::lcd::print(1,"%.1lf", current_l_tl_error);

        // velocity pid: based on the rate of change of velocity, pid updates the power the wheels 
        l_velocity_pid += left_velocity_PID.step(current_l_tl_error); 
        r_velocity_pid += right_velocity_PID.step(current_r_tl_error); 

        // angle pid: based on error, pid updates the power to the wheels 
        l_angle_pid = left_angle_PID.step(l_error); //power to force anticlockwise aiming
        r_angle_pid = right_angle_PID.step(r_error); 

        // higher base_v: drifts and lower base_v: lags 
        lscale = (battery_voltage/MAX_VOLTAGE) * scale; // * ((1.0-base_v)*fabs((l_error))+base_v); 
        rscale = (battery_voltage/MAX_VOLTAGE) * scale; // * ((1.0-base_v)*fabs((r_error))+base_v); 

        // lu = (int32_t)std::clamp(lscale * (l_velocity_pid + l_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE); //this side seems less powerful on the robot 
        // ll = (int32_t)std::clamp(lscale * (l_velocity_pid - l_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE);
        // ru = (int32_t)std::clamp(rscale * (r_velocity_pid + r_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE);
        // rl = (int32_t)std::clamp(rscale * (r_velocity_pid - r_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE);

        lu = (int32_t)(lscale * (l_velocity_pid + l_angle_pid));
        ll = (int32_t)(lscale * (l_velocity_pid - l_angle_pid));
        ru = (int32_t)(rscale * (r_velocity_pid + r_angle_pid));
        rl = (int32_t)(rscale * (r_velocity_pid - r_angle_pid));

        //pros::lcd::print(4,"l_velocity_pid: %.1lf", l_velocity_pid);
        //pros::lcd::print(5,"r_velocity_pid: %.1lf", r_velocity_pid);

        pros::lcd::print(6, "lu:%.1d,ll:%.1d", lu, ll);
        pros::lcd::print(7, "ru:%.1d,rl:%.1d", ru, rl);

        move_voltage_wheels(lu,ll,ru,rl);
        pros::delay(5);
    }
}

void autonomous(){
    moveBaseAutonomous(0.0, 250.0);
    pros::delay(50);
    moveBaseAutonomous(0.0, -250.0);
}

void initialize(){
    pros::lcd::initialize();
    pros::delay(50);
    master.clear();
    while(!imu.reset(true));  //uncomment for actual
    //pros::delay(100);
    //master.print(0,0,"IMU calibrated  ");
    pros::delay(100);
    master.rumble(" . . .");

    setBrakeModes();

    //while(!left_rotation_sensor.reset());
    //while(!right_rotation_sensor.reset());

    left_rotation_sensor.set_data_rate(5);
    right_rotation_sensor.set_data_rate(5);

    left_rotation_sensor.set_position(0);
    right_rotation_sensor.set_position(0);

    //defaultSlamValue = slam_dunk.get_value();

    pros::Task move_base(moveBase);
    pros::Task slam_dunk(slamDunk);
    pros::Task serial_read(serialRead);
}


void opcontrol(){
    while(true){
        leftX = master.get_analog(ANALOG_LEFT_X);
        leftY = master.get_analog(ANALOG_LEFT_Y);
        rightX = master.get_analog(ANALOG_RIGHT_X);
        rightY = master.get_analog(ANALOG_RIGHT_Y);
        if(master.get_digital_new_press(DIGITAL_B)) autonomous();

        if(master.get_digital_new_press(DIGITAL_Y)) driver = !driver;

        pros::lcd::print(5,"pos: %.2f, %%: %.3f, prx: %d", conveyor.get_position(), conveyor.get_position()/conveyor_loop_period, conveyor_optical.get_proximity());

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) { 
            // pros::lcd::print(0, "R1 pressed, CONVEYOR FORWARD\n");
            // conveyor.move(110); 
            step_conveyor();

            } 
        else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
            // pros::lcd::print(0, "R2 pressed, CONVEYOR BACKWARD\n");
            // conveyor.move(-110);
            calibrate_conveyor();
            }
        else { 
            //pros::lcd::print(0, "CONVEYOR STOPPED\n");
            //pros::lcd::print(0, "CONVEYOR STOPPED\n");
            conveyor.move(0);
        }

        // L1 FORWARD, L2 BACKWARD FOR ROLLER (missing hardware)
        // when L1 is pressed, rollers move forward with NEGATIVE velocity??
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { 
            //pros::lcd::print(0, "L2: ROLLER backward, +ve velocity??\n");
            roller.move(110); 
        } 
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { 
        // detect ring and move conveyor later
		roller.move(-110);
        check_for_ring();
        } 
    else {
		roller.move(0);
        
        // basically wait for a bit, along with check_for_ring(), this can be in task
        if (conveyor_step == 1 && detected_ring_time >= 1){
            detected_ring_time--; // wait for conveyor to get to position
            conveyor_optical.set_led_pwm(100);
        }
        else if (conveyor_step == 1){
            // conveyor at "store", now check colour (ONLY IN AUTON)
            is_ring_ours = same_colour();
        }
        else{
            conveyor_optical.set_led_pwm(0);
        }
    }
    pros::lcd::print(0, "ring: %s", is_ring_ours ? "same" : "not same");
    pros::delay(5);
    }
}