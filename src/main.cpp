#include "main.h"

// Function to determine sign of a integer variable, returns bool
template <typename T>
int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

double check_sign(double num)
{
    if (num < 0.0)
        return -1.0;
    else if (num > 0.0)
        return 1.0;
    else
        return 0.0;
}

void setBrakeModes()
{
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

void setMotorCurrentLimit(int current)
{
    luA.set_current_limit(current);
    luB.set_current_limit(current);
    llA.set_current_limit(current);
    llB.set_current_limit(current);
    ruA.set_current_limit(current);
    ruB.set_current_limit(current);
    rlA.set_current_limit(current);
    rlB.set_current_limit(current);
}

void serialRead(void *params)
{
    if(!tasks_enabled) return;
    vexGenericSerialEnable(SERIALPORT - 1, 0);
    vexGenericSerialBaudrate(SERIALPORT - 1, 115200);
    pros::delay(5);
    pros::screen::set_pen(COLOR_RED);
    double dist_X = 0.0;
    double dist_Y = 0.0;
    double prevDist_x = 0.0;
    double prevDist_y = 0.0;
    uint8_t buffer[256];
    int bufLength = 256;
    while (true)
    {
        if(pros::millis() > (auton_time + max_auton_time)){
            break;
        }
            int32_t nRead = vexGenericSerialReceive(SERIALPORT - 1, buffer, bufLength);
            if (nRead >= 0)
            {
                std::stringstream dataStream("");
                bool recordOpticalX = false;
                bool recordOpticalY = false;
                for (int i = 0; i < nRead; i++)
                {
                    char thisDigit = (char)buffer[i];
                    if (thisDigit == 'D' || thisDigit == 'I' || thisDigit == 'A' || thisDigit == 'X' || thisDigit == 'C' || thisDigit == 'Y')
                    {
                        recordOpticalX = false;
                        recordOpticalY = false;
                    }
                    if (thisDigit == 'C')
                    {
                        recordOpticalX = false;
                        dataStream >> dist_X;
                        global_distX = dist_X * -10.0;
                        dataStream.str(std::string());
                        prevDist_x = dist_X * -10.0;
                    }
                    if (thisDigit == 'D')
                    {
                        recordOpticalY = false;
                        dataStream >> dist_Y;
                        global_distY = dist_Y * -10.0;
                        dataStream.str(std::string());
                        prevDist_y = dist_Y * -10.0;
                    }
                    if (recordOpticalX)
                        dataStream << (char)buffer[i];
                    if (recordOpticalY)
                        dataStream << (char)buffer[i];
                    if (thisDigit == 'X')
                        recordOpticalX = true;
                    if (thisDigit == 'Y')
                        recordOpticalY = true;
                }
                pros::lcd::print(6,"%.1lf",global_distY);
            }
            pros::Task::delay(2);
    }
}

void brake()
{ // brakes all base motors
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

void tareBaseMotorEncoderPositions() // tares all base motor encoder positions
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

void clampVoltage(int32_t VOLTAGE)
{
    // if any of lu, ll, ru or rl are too big, we need to scale them, and we must scale them all by the same amount so we dont throw off the proportions
    if (abs(lu) > VOLTAGE || abs(ll) > VOLTAGE || abs(ru) > VOLTAGE || abs(rl) > VOLTAGE)
    {
        // figure out which of lu, ll, ru or rl has the largest magnitude
        int32_t max = abs(lu);
        if (max < abs(ll))
            max = abs(ll);
        if (max < abs(ru))
            max = abs(ru);
        if (max < abs(rl))
            max = abs(rl);

        double VoltageScalingFactor = ((double)max) / VOLTAGE; // this will definitely be positive, hence it wont change the sign of lu, ll, ru or rl.
        lu = (int32_t)((double)lu / VoltageScalingFactor);
        ll = (int32_t)((double)ll / VoltageScalingFactor);
        ru = (int32_t)((double)ru / VoltageScalingFactor);
        rl = (int32_t)((double)rl / VoltageScalingFactor);
        // master.print(0,0,"%6d",lu);
    }
}

void limitVoltage(int32_t BATTERY_VOLTAGE)
{
    if (fabs(lu) < VOLTAGE_CUTOFF)
    {
        lu = 0;
    }
    else
    {
        lu = (lu * ((BATTERY_VOLTAGE - VOLTAGE_CUTOFF) / BATTERY_VOLTAGE)) + VOLTAGE_CUTOFF;
    }
    if (fabs(ll) < VOLTAGE_CUTOFF)
    {
        ll = 0;
    }
    else
    {
        ll = (ll * ((BATTERY_VOLTAGE - VOLTAGE_CUTOFF) / BATTERY_VOLTAGE)) + VOLTAGE_CUTOFF;
    }
    if (fabs(ru) < VOLTAGE_CUTOFF)
    {
        ru = 0;
    }
    else
    {
        ru = (ru * ((BATTERY_VOLTAGE - VOLTAGE_CUTOFF) / BATTERY_VOLTAGE)) + VOLTAGE_CUTOFF;
    }
    if (fabs(rl) < VOLTAGE_CUTOFF)
    {
        rl = 0;
    }
    else
    {
        rl = (rl * ((BATTERY_VOLTAGE - VOLTAGE_CUTOFF) / BATTERY_VOLTAGE)) + VOLTAGE_CUTOFF;
    }
}

void move_voltage_wheels(int32_t lu, int32_t ll, int32_t ru, int32_t rl)
{
    luA.move_voltage(lu);
    luB.move_voltage(lu);

    llA.move_voltage(ll);
    llB.move_voltage(ll);

    ruA.move_voltage(ru);
    ruB.move_voltage(ru);

    rlA.move_voltage(rl);
    rlB.move_voltage(rl);
}

double wrapAngle(double angle)
{ // forces the angle to be within the -180 < angle < 180 range
    if (angle > 180.0)
        while (angle > 180.0)
            angle -= 360.0;
    else if (angle < -180.0)
        while (angle < -180.0)
            angle += 360.0;
    return angle;
}

double getNormalizedSensorAngle(pros::Rotation &sensor)
{                                              // Converts rotational sensor readings into degrees and bounds it between -180 to 180
    double angle = sensor.get_angle() / 100.0; // Convert from centidegrees to degrees
    return wrapAngle(angle);                   // forces the angle to be within the -180 < angle < 180 range
}

vector3D normalizeJoystick(int x_in, int y_in)
{                                                  // convert translation joystick input to polar vector
    double angle = atan2(y_in, x_in) * TO_DEGREES; // angle of translation polar vector. Note that atan2 automatically considers the sign to deal with the trigonometry quadrant
    double scaleLength;
    double magnitude; // magnitude of translation polar vector
    double length = sqrt(x_in * x_in + y_in * y_in);
    vector3D out;
    if (length < DEADBAND)
    {                            // if the joystick is too close to the origin, dont bother moving (this is to correct for stick drift, where the joystick doesnt default to the 0,0 position due to physical damage)
        out.load(0.0, 0.0, 0.0); // assign zero values to the xyz attributes of the vector3D named "out"
        return out;
    }
    // forcing the joystick output to be a circle not the square bounding box of the joystick
    // for any radial line of the circle, we find its length from the deadband radius to the radius of the circle of the joystick, then map the speed from 0 to 1 of that length
    // this means that in any direction, we can reach our full range of speeds without being limited
    // use CSC or SEC as required
    if ((angle > 45.0 && angle < 135.0) || (angle < -45.0 && angle > -135.0))
        scaleLength = 127.0 / sin(angle * TO_RADIANS);
    else
        scaleLength = 127.0 / cos(angle * TO_RADIANS);

    scaleLength = fabs(scaleLength) - DEADBAND;    // force scaleLength to be positive and subtract deadband
    magnitude = (length - DEADBAND) / scaleLength; // find magnitude of translation vector and scale it down (note that magnitude will always be positive)

    // assign values to the xyz attributes of the vector3D named "out"
    out.load(magnitude * cos(angle * TO_RADIANS), magnitude * sin(angle * TO_RADIANS), 0.0);
    return out;
}

vector3D normalizeRotation(int x_in)
{ // get rotation speed from rotation joystick
    vector3D out;
    double scaleLength = 127.0 - DEADBAND;
    if (abs(x_in) < DEADBAND)
    {                            // if the joystick is too close to the origin, dont bother moving (this is to correct for stick drift, where the joystick doesnt default to the 0,0 position due to physical damage)
        out.load(0.0, 0.0, 0.0); // assign values to the xyz attributes of the vector3D named "out"
        return out;
    }
    double value = (abs(x_in) - DEADBAND) / scaleLength; // find magnitude of rotation and scale it down
    if (x_in > 0)
    {
        value = value * -1.0; // Note that positive x corresponds to negative rotation speed as rotation speed is anticlockwise.
    }
    // this is SUPPOSED to be a vector, its not wrong
    // both normaliseRotation and normaliseJoystick return a vector for standardisation. This is intended behaviour.
    out.load(0.0, 0.0, value); // assign values to the xyz attributes of the vector3D named "out"
    return out;
}

double angle(vector3D v1, vector3D v2)
{
    double dot = v1 * v2;
    double det = v1.x * v2.y - v1.y * v2.x;
    return -1.0 * atan2(det, dot); // atan2 automatically considers the sign to deal with the trigonometry quadrant
}

double max(double a, double b)
{ // returns the larger of two doubles
    return (a > b) ? a : b;
}

double min(double a, double b)
{ // returns the smaller of two doubles
    return (a < b) ? a : b;
}

// Driver code
void moveBase(void *params)
{
    double v_right_velocity; // target velocity magnitude
    double v_left_velocity;

    double battery_voltage;

    double left_angle;
    double right_angle;
    double left_target_angle;
    double right_target_angle;
    // vector3D rotational_v_vector; // vector expression for angular velocity -- only has z component

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

    vector3D current_tl_velocity(0, 0, 0); // current transaltional velocity

    vector3D prev_target_v(0, 0, 0);
    vector3D prev_target_r(0, 0, 0);

    vector3D v_fterm(0, 0, 0); // fterm: added constants to make PID more responsive
    vector3D r_fterm(0, 0, 0); // based on rate of change of input from joystick

    double average_x_v = 0;
    double average_y_v = 0;

    uint64_t micros_now = -1;

    uint64_t micros_prev = pros::micros();
    uint64_t dt = -1;

    // voltages
    int32_t lu = 0; // left upper
    int32_t ll = 0; // left lower
    int32_t ru = 0; // right upper
    int32_t rl = 0; // right lower

    // PID instances
    PID left_angle_PID(angle_kP_left, angle_kI_left, angle_kD_left);
    PID right_angle_PID(angle_kP_right, angle_kI_right, angle_kD_right);
    PID left_velocity_PID(velocity_kP, velocity_kI, velocity_kD);
    PID right_velocity_PID(velocity_kP, velocity_kI, velocity_kD);
    PID rotate_robot_PID(azim_kP, azim_kI, azim_kD);

    vector3D L2I_pos(WHEEL_BASE_RADIUS, 0.0, 0.0);
    vector3D imu_angular;
    vector3D angular_error;
    vector3D rot_pid;
    vector3D rot_FF;

    double rot_vector_double = 0.0;
    double rot_pid_double = 0.0;
    double gyro_rate = 0.0;
    double imu1_rate = 0.0;
    double imu2_rate = 0.0;
    double a_err_d = 0.0; // angular error as a double

    while (true)
    {
        prev_target_v = target_v; // prev target velocity
        prev_target_r = target_r; // prev target rotation

        target_v = normalizeJoystick(leftX, leftY).scalar(MAX_SPEED); // target velocity
        // to be updated: leftX = 0 to remove left and right translations
        target_r = normalizeRotation(rightX).scalar(MAX_ANGULAR * MAX_ANGULAR_SCALE); // target rotation
        // imu1_gyro = imu.get_gyro_rate().z * -1.0 * TO_RADIANS;
        // imu2_gyro = imu2.get_gyro_rate().z * 1.0 * TO_RADIANS;
        left_angle = wrapAngle(getNormalizedSensorAngle(left_rotation_sensor) - 90.0) * TO_RADIANS;   // takes robot right as 0
        right_angle = wrapAngle(getNormalizedSensorAngle(right_rotation_sensor) - 90.0) * TO_RADIANS; // Y axis positive is front

        current_left_vector = vector3D(cos(left_angle), sin(left_angle), 0.0);
        current_right_vector = vector3D(cos(right_angle), sin(right_angle), 0.0);

        current_l_velocity = ((luA.get_actual_velocity() + luB.get_actual_velocity() + llA.get_actual_velocity() + llB.get_actual_velocity()) / 4.0);
        current_r_velocity = ((ruA.get_actual_velocity() + ruB.get_actual_velocity() + rlA.get_actual_velocity() + rlB.get_actual_velocity()) / 4.0);

        current_angular = (-current_l_velocity * sin(left_angle) + current_r_velocity * sin(right_angle)) / (2.0 * WHEEL_BASE_RADIUS); // current angular velocity
        // average_x_v = ((current_l_velocity*cos(left_angle))+(current_r_velocity*cos(right_angle)))/2.0;
        // average_y_v = ((current_l_velocity*sin(left_angle))+(current_r_velocity*sin(right_angle)))/2.0;
        // current_tl_velocity.load(average_x_v,average_y_v,0.0);

        // if(imu.is_calibrating()/*||imu2.is_calibrating()*/){
        //     gyro_rate = current_angular;    // ignore gyro while calibrating, use encoder values
        //     master.rumble(".");
        // }else{
        //     gyro_rate = -1.0 * imu.get_gyro_rate().z * TO_RADIANS;
        // }

        gyro_rate = -1.0 * imu.get_gyro_rate().z * TO_RADIANS;

        imu_angular = vector3D(0.0, 0.0, gyro_rate); // Radians per second, loaded as vector

        battery_voltage = pros::battery::get_voltage();
        if (battery_voltage > MAX_VOLTAGE)
        {
            battery_voltage = MAX_VOLTAGE;
        }

        micros_prev = micros_now;
        micros_now = pros::micros();
        dt = micros_now - micros_prev;
        v_fterm = (target_v - prev_target_v) * (v_kF / dt); // rate of change of joystick input * constant v_kF
        r_fterm = (target_r - prev_target_r) * (r_kF / dt); // rate of change of joystick input * constant r_kF
        target_v = target_v + v_fterm;                      // update the target_v and target_r
        target_r = target_r + r_fterm;

        angular_error = target_r - imu_angular;

        if (fabs(angular_error.z) < ANGULAR_THRESH || abs(rightY) > 90)
        { // right stick up or slow rotation
            angular_error.load(0.0, 0.0, 0.0);
            rot_pid_double = 0.0;
        }

        a_err_d = angular_error.getZ();
        rot_pid_double += rotate_robot_PID.step(a_err_d);

        rot_FF = (target_r ^ L2I_pos).scalar(r_kF_STATIC);
        rot_vector_double = rot_pid_double + rot_FF.getY();
        rot_pid = vector3D(0.0, rot_vector_double, 0.0);

        v_left = target_v - rot_pid; // in order to rotate counterclockwise
        v_right = target_v + rot_pid;

        bool reverse_right = false;
        bool reverse_left = false;

        // check if the angle is obtuse
        if (v_left * current_left_vector < 0)
        {
            // reverse if angle is obtuse for shorter rotation
            v_left = -v_left;
            reverse_left = true;
        }

        if (v_right * current_right_vector < 0)
        {
            // reverse if angle is obtuse for shorter rotation
            v_right = -v_right;
            reverse_right = true;
        }

        v_right_velocity = SPEED_TO_RPM * TRANSLATE_RATIO * (v_right * current_right_vector); // dot product should already
        v_left_velocity = SPEED_TO_RPM * TRANSLATE_RATIO * (v_left * current_left_vector);    // compensate angle drift?

        if (reverse_left)
        {
            v_left_velocity = -v_left_velocity;
        }

        if (reverse_right)
        {
            v_right_velocity = -v_right_velocity;
        }

        // calculate the error angle
        if (target_v.norm() > 0.1 || target_r.norm() > 0.01)
        {
            l_error = angle(v_left, current_left_vector);
            r_error = angle(v_right, current_right_vector);
        }
        else
        {                  // NOT TL NOT ROTATE
            l_error = 0.0; // DO NOT CHANGE WHEEL ANGLE IF NOT MOVING WHEEL
            r_error = 0.0; // THIS IS FOR BETTER BRAKING PERFORMANCE
        }

        if (std::isnan(l_error) || std::isnan(r_error))
        {
            l_error = 0.0;
            r_error = 0.0;
        }

        // calculate the wheel error
        current_l_tl_error = (v_left_velocity - current_l_velocity);
        current_r_tl_error = (v_right_velocity - current_r_velocity);

        // velocity pid: based on the rate of change of velocity, pid updates the power the wheels
        l_velocity_pid += left_velocity_PID.step(current_l_tl_error);
        r_velocity_pid += right_velocity_PID.step(current_r_tl_error);

        // angle pid: based on error, pid updates the power to the wheels
        l_angle_pid = left_angle_PID.step(l_error); // power to force anticlockwise aiming
        r_angle_pid = right_angle_PID.step(r_error);

        // higher base_v: drifts and lower base_v: lags
        lscale = (battery_voltage / MAX_VOLTAGE) * scale; // * ((1.0-base_v)*fabs((l_error))+base_v);
        rscale = (battery_voltage / MAX_VOLTAGE) * scale; // * ((1.0-base_v)*fabs((r_error))+base_v);

        // lu = (int32_t)std::clamp(lscale * (l_velocity_pid + l_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE); //this side seems less powerful on the robot
        // ll = (int32_t)std::clamp(lscale * (l_velocity_pid - l_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE);
        // ru = (int32_t)std::clamp(rscale * (r_velocity_pid + r_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE);
        // rl = (int32_t)std::clamp(rscale * (r_velocity_pid - r_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE);

        lu = (int32_t)lscale * (l_velocity_pid + l_angle_pid);
        ll = (int32_t)lscale * (l_velocity_pid - l_angle_pid);
        ru = (int32_t)rscale * (r_velocity_pid + r_angle_pid);
        rl = (int32_t)rscale * (r_velocity_pid - r_angle_pid);
        clampVoltage(battery_voltage);
        limitVoltage(battery_voltage);

        std::cout << lu << " : " << ll << " : " << ru << " : " << rl << std::endl;
        move_voltage_wheels(lu, ll, ru, rl);
        pros::delay(2);
    }
}

// Helper function to check if the motor is at the target
bool isMotorAtTarget(int port, int target)
{
    int currentPosition = pros::c::motor_get_position(port);
    return (currentPosition >= target - 5) && (currentPosition <= target + 5);
}

// Arms code
// Arms control function
void slamDunk(void *params)
{
    double Derivative = 0.0;
    double prevError = 0.0;
    double Error = 0.0;
    double Integral = 0.0;

    // defaultSlamValue = slam_dunk.get_value();
    // master.print(3,0,"%d", defaultSlamValue);
    while (true)
    {
        switch (slammingState)
        {
        case SLAM_START_STATE: // resting position
            slam_target = defaultSlamValue;
            break;
        case SLAM_MID_STATE: // midpoint - holding position
            slam_target = defaultSlamValue - 195;
            break;
        case SLAM_EXTENDED_STATE: // extended all the way
            slam_target = defaultSlamValue - 1555;
            break;
        case SLAM_LADDER: // extended all the way
            slam_target = defaultSlamValue - 1100;
        default:
            slam_target = defaultSlamValue;
            break;
        }

        Derivative = prevError - Error;
        Error = fabs(slam_target - slam_dunk.get_value());
        Integral += Error;
        double motorPower = slam_Kp * Error + slam_Kd * Derivative + slam_Ki * Integral;

        if (fabs(Error) <= 5.0)
        {
            slam_dunk_motor.move(0);
            slam_dunk_motor.brake();
        }
        else
        {
            if (slam_target > slam_dunk.get_value() + 10.0)
            {
                slam_dunk_motor.move(motorPower);
            }
            else if (slam_target < slam_dunk.get_value() - 10.0)
            {
                slam_dunk_motor.move(-motorPower);
            }
        }
        prevError = Error;
        pros::Task::delay(15);
    }
}

void moveBaseAutonomous(double targetX, double targetY, double target_heading)
{
    targetY *= -1.0;
    if (((fabs(targetX) < 100.0) && (fabs(targetY) < 100.0)))
    {
        auton_distance_kP = 0.3; // swerve wheel rotation distance
        auton_distance_kI = 0.0;
        auton_distance_kD = 20.0; // 20 was
    }
    else if (fabs(targetX) > 400.0 || fabs(targetY) > 400.0)
    {
        auton_distance_kP = 0.12; // swerve wheel rotation distance
        auton_distance_kI = 0.0;
        auton_distance_kD = 2.;
    }
    else
    {
        auton_distance_kP = 0.1; // swerve wheel rotation distance
        auton_distance_kI = 0.0;
        auton_distance_kD = 0.02;
    }

    // if(fabs(targetX) > 800.0 || fabs(targetY) >800.0){
    //     auton_distance_kP = 0.1; //swerve wheel rotation distance
    //     auton_distance_kI = 0.0;
    //     auton_distance_kD = 0.0; // 20 was
    // }

    double v_right_velocity = 0.0; // target velocity magnitude
    double v_left_velocity = 0.0;

    double battery_voltage;

    double left_angle;
    double right_angle;
    double left_target_angle;
    double right_target_angle;
    // vector3D rotational_v_vector; // vector expression for angular velocity -- only has z component

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

    vector3D current_tl_velocity(0, 0, 0); // current transaltional velocity

    vector3D prev_target_v(0, 0, 0);
    vector3D prev_target_r(0, 0, 0);

    vector3D v_fterm(0, 0, 0); // fterm: added constants to make PID more responsive
    vector3D r_fterm(0, 0, 0); // based on rate of change of input from joystick

    double average_x_v = 0;
    double average_y_v = 0;

    uint64_t micros_now = -1;

    uint64_t micros_prev = pros::micros();
    uint64_t dt = -1;

    // voltages
    int32_t lu = 0; // left upper
    int32_t ll = 0; // left lower
    int32_t ru = 0; // right upper
    int32_t rl = 0; // right lower

    // PID instances
    PID left_angle_PID(auton_angle_kP_left, auton_angle_kI_left, auton_angle_kD_left);
    PID right_angle_PID(auton_angle_kP_right, auton_angle_kI_right, auton_angle_kD_right);
    PID left_velocity_PID(auton_l_velocity_kP, auton_l_velocity_kI, auton_l_velocity_kD);
    PID right_velocity_PID(auton_r_velocity_kP, auton_r_velocity_kI, auton_r_velocity_kD);

    if (fabs(target_heading) > 0.0)
    {
        auton_azim_kP = 0.03; // azimuth, for correcting rotation
        auton_azim_kI = 0.0;  // drunk
        auton_azim_kD = 10.0;

        AUTON_ANGULAR_THRESH = 0.005; // Threshold under which to ignore angular error
    }
    else
    {
        // auton_azim_kP = 0.7; //azimuth, for correcting rotation //works for translate left or right
        // auton_azim_kI = 0.0;    //drunk
        // auton_azim_kD = 15.0;

        // auton_azim_kP = 0.18; // azimuth, for correcting rotation
        // auton_azim_kI = 0.0;  // drunk
        // auton_azim_kD = 10.0;

        auton_azim_kP = 0.05; // azimuth, for correcting rotation
        auton_azim_kI = 0.0;  // drunk
        auton_azim_kD = 10.0;

        AUTON_ANGULAR_THRESH = 0.00; // Threshold under which to ignore angular error
    }
    PID rotate_robot_PID(auton_azim_kP, auton_azim_kI, auton_azim_kD);

    PID delta_X_PID(auton_distance_kP, auton_distance_kI, auton_distance_kD);
    PID delta_Y_PID(auton_distance_kP, auton_distance_kI, auton_distance_kD);

    PID delta_Heading_PID(auton_heading_kP, auton_heading_kI, auton_heading_kD);

    vector3D L2I_pos(WHEEL_BASE_RADIUS, 0.0, 0.0);
    vector3D imu_angular;
    vector3D angular_error;
    vector3D rot_pid;
    vector3D rot_FF;

    double rot_vector_double = 0.0;
    double rot_pid_double = 0.0;
    double gyro_rate = 0.0;
    double a_err_d = 0.0; // angular error as a double

    double errorX = 0.0;
    double errorY = 0.0;
    double errorheading = 0.0;

    double target_v_x;
    double target_v_y;
    double target_r_heading;

    double offsetX = global_distX;
    double offsetY = global_distY;
    // pros::lcd::print(6, "offset_y:%.1lf", offsetY);
    if (fabs(target_heading) > 0.0)
        while (imu.tare_rotation() == PROS_ERR);

    while (true)
    {
        if(pros::millis() > (auton_time + max_auton_time)){
            break;
        }
        prev_target_v = target_v; // prev target velocity
        prev_target_r = target_r; // prev target rotation
        // if(fabs(targetX) > 0.0)
        //     errorX = fabs(fabs(targetX) - fabs((fabs(global_distX) - offsetX)));
        // if(fabs(targetX) > 0.0 && fabs(errorX) > fabs(targetX))
        //     errorX = 0.0;
        // if(fabs(targetX) <= 0.0 || fabs(errorX) <= 3.0)
        //     errorX = 0.0;
        // if(fabs(fabs(global_distX) - offsetX) > fabs(targetX))
        //     errorX = 0.0;

        if (fabs(targetX) != 0.0)
            errorX = targetX - (global_distX - offsetX);
        else
            errorX = 0.0;
        if (errorX < 2.0 && errorX > -2.0)
            errorX = 0.0;

        if (fabs(targetY) != 0.0)
            errorY = targetY - (global_distY - offsetY);
        else
            errorY = 0.0;
        if (errorY < 2.0 && errorY > -2.0)
            errorY = 0.0;
        pros::lcd::print(1, "err_x: %1.1lf, err_y: %1.1lf", errorX, errorY);
        if (fabs(target_heading) > 0.0)
            errorheading = fabs(fabs(target_heading) - fabs(imu.get_rotation()));
        if (fabs(target_heading) <= 0.0 || fabs(errorheading) <= 2.0)
            errorheading = 0.0;
        if (fabs(errorheading) > fabs(target_heading))
            errorheading = 0.0;
        if (fabs(target_heading) > 0.0 && fabs(errorheading) > fabs(target_heading))
            errorheading = 0.0;

        if (fabs(errorX) <= 3.0 && fabs(errorY) <= 3.0 && fabs(errorheading) <= 0.5)
        {
            move_voltage_wheels(0, 0, 0, 0);
            lu = 0;
            ll = 0;
            ru = 0;
            rl = 0;
            brake();
            break;
        }

        target_v_x = delta_X_PID.step(errorX);
        target_v_y = delta_Y_PID.step(errorY);

        target_r_heading = delta_Heading_PID.step(errorheading);
        // pros::lcd::print(4,"target_v_x: %.2lf", target_v_y);

        target_v = vector3D(target_v_x, -target_v_y, 0.0);                                                      // target velocity
        target_r = normalizeRotation(check_sign(target_heading) * target_r_heading).scalar(MAX_ANGULAR * 0.35); // target rotation

        // pros::lcd::print(1,"target_v_y: %.2lf", target_v_y);
        // pros::lcd::print(2,"error_y: %.2lf",errorY);
        // pros::lcd::print(3,"global_y: %.2lf",global_distY);
        // pros::lcd::print(4,"global_y - offset: %.2lf",(global_distX - offsetX));
        // pros::lcd::print(5,"target_y: %.2lf", targetY);

        // takes robot right as 0
        // Y axis positive is front
        left_angle = wrapAngle(getNormalizedSensorAngle(left_rotation_sensor) - 90.0) * TO_RADIANS;
        right_angle = wrapAngle(getNormalizedSensorAngle(right_rotation_sensor) - 90.0) * TO_RADIANS;

        current_left_vector = vector3D(cos(left_angle), sin(left_angle), 0.0);
        current_right_vector = vector3D(cos(right_angle), sin(right_angle), 0.0);

        current_l_velocity = ((luA.get_actual_velocity() + luB.get_actual_velocity() + llA.get_actual_velocity() + llB.get_actual_velocity()) / 4.0);
        current_r_velocity = ((ruA.get_actual_velocity() + ruB.get_actual_velocity() + rlA.get_actual_velocity() + rlB.get_actual_velocity()) / 4.0);

        current_angular = (-current_l_velocity * sin(left_angle) + current_r_velocity * sin(right_angle)) / (2.0 * WHEEL_BASE_RADIUS); // current angular velocity
        // average_x_v = ((current_l_velocity*cos(left_angle))+(current_r_velocity*cos(right_angle)))/2.0;
        // average_y_v = ((current_l_velocity*sin(left_angle))+(current_r_velocity*sin(right_angle)))/2.0;
        // current_tl_velocity.load(average_x_v,average_y_v,0.0);

        // if(imu.is_calibrating()){
        //     gyro_rate = current_angular;    // ignore gyro while calibrating, use encoder values
        // }else{
        //     gyro_rate = -1.0 * imu.get_gyro_rate().z * TO_RADIANS;
        // }

        gyro_rate = -1.0 * imu.get_gyro_rate().z * TO_RADIANS;

        imu_angular = vector3D(0.0, 0.0, gyro_rate); // Radians per second, loaded as angle

        battery_voltage = pros::battery::get_voltage();

        micros_prev = micros_now;
        micros_now = pros::micros();
        dt = micros_now - micros_prev;
        v_fterm = (target_v - prev_target_v) * (v_kF / dt); // rate of change of joystick input * constant v_kF
        r_fterm = (target_r - prev_target_r) * (r_kF / dt); // rate of change of joystick input * constant r_kF
        target_v = target_v + v_fterm;                      // update the target_v and target_r
        target_r = target_r + r_fterm;

        angular_error = target_r - imu_angular;

        if (fabs(angular_error.z) < AUTON_ANGULAR_THRESH)
        {
            angular_error.load(0.0, 0.0, 0.0);
            rot_pid_double = 0.0;
        }

        a_err_d = angular_error.getZ();
        rot_pid_double += rotate_robot_PID.step(a_err_d);

        rot_FF = (target_r ^ L2I_pos).scalar(r_kF_STATIC);
        rot_vector_double = rot_pid_double + rot_FF.getY();
        rot_pid = vector3D(0.0, rot_vector_double, 0.0);

        v_left = target_v - rot_pid; // in order to rotate counterclockwise
        v_right = target_v + rot_pid;

        bool reverse_right = false;
        bool reverse_left = false;

        // check if the angle is obtuse
        if (v_left * current_left_vector < 0)
        {
            // reverse if angle is obtuse for shorter rotation
            v_left = -v_left;
            reverse_left = true;
        }

        if (v_right * current_right_vector < 0)
        {
            // reverse if angle is obtuse for shorter rotation
            v_right = -v_right;
            reverse_right = true;
        }

        // in RPM
        v_right_velocity = SPEED_TO_RPM * TRANSLATE_RATIO * (v_right * current_right_vector); // dot product should already
        v_left_velocity = SPEED_TO_RPM * TRANSLATE_RATIO * (v_left * current_left_vector);    // compensate angle drift?

        if (reverse_left)
        {
            v_left_velocity = -v_left_velocity;
        }

        if (reverse_right)
        {
            v_right_velocity = -v_right_velocity;
        }

        // calculate the error angle
        // l_error = angle(v_left, current_left_vector);
        // r_error = angle(v_right, current_right_vector);
        if (target_v.norm() > 0.1 || target_r.norm() > 0.01)
        {
            l_error = angle(v_left, current_left_vector);
            r_error = angle(v_right, current_right_vector);
        }
        else
        {                  // NOT TL NOT ROTATE
            l_error = 0.0; // DO NOT CHANGE WHEEL ANGLE IF NOT MOVING WHEEL
            r_error = 0.0; // THIS IS FOR BETTER BRAKING PERFORMANCE
        }

        if (std::isnan(l_error) || std::isnan(r_error))
        {
            l_error = 0.0;
            r_error = 0.0;
        }

        // calculate the wheel error
        current_l_tl_error = (v_left_velocity - current_l_velocity);
        current_r_tl_error = (v_right_velocity - current_r_velocity);
        // pros::lcd::print(1,"%.1lf", current_l_tl_error);

        // velocity pid: based on the rate of change of velocity, pid updates the power the wheels
        l_velocity_pid += left_velocity_PID.step(current_l_tl_error);
        r_velocity_pid += right_velocity_PID.step(current_r_tl_error);

        // angle pid: based on error, pid updates the power to the wheels
        l_angle_pid = left_angle_PID.step(l_error); // power to force anticlockwise aiming
        r_angle_pid = right_angle_PID.step(r_error);

        // higher base_v: drifts and lower base_v: lags
        // lscale = (battery_voltage/MAX_VOLTAGE) * scale; // * ((1.0-base_v)*fabs((l_error))+base_v);
        // rscale = (battery_voltage/MAX_VOLTAGE) * scale; // * ((1.0-base_v)*fabs((r_error))+base_v);

        lscale = (battery_voltage / MAX_VOLTAGE) * scale; // * ((1.0-base_v)*fabs((l_error))+base_v);
        rscale = (battery_voltage / MAX_VOLTAGE) * scale;

        // lu = (int32_t)std::clamp(lscale * (l_velocity_pid + l_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE); //this side seems less powerful on the robot
        // ll = (int32_t)std::clamp(lscale * (l_velocity_pid - l_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE);
        // ru = (int32_t)std::clamp(rscale * (r_velocity_pid + r_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE);
        // rl = (int32_t)std::clamp(rscale * (r_velocity_pid - r_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE);

        lu = (int32_t)(lscale * (l_velocity_pid + l_angle_pid));
        ll = (int32_t)(lscale * (l_velocity_pid - l_angle_pid));
        ru = (int32_t)(rscale * (r_velocity_pid + r_angle_pid));
        rl = (int32_t)(rscale * (r_velocity_pid - r_angle_pid));
        clampVoltage(battery_voltage);
        limitVoltage(battery_voltage);
        // move_voltage_wheels(0,0,0,0);
        //  pros::lcd::print(2,"lu:%d, ru:%d",lu,ru);
        //  pros::lcd::print(3,"ll:%d, rl:%d",ll,rl);
        move_voltage_wheels(lu, ll, ru, rl);
        pros::delay(3);
    }
}

void alignWheels(vector3D heading)
{
    double left_angle;
    double right_angle;
    double left_target_angle;
    double right_target_angle;
    // vector3D rotational_v_vector; // vector expression for angular velocity -- only has z component

    vector3D current_left_vector; // direction unit vector for wheels
    vector3D current_right_vector;

    double l_error = 0.0; // how far the left and right angles wrt to the their respective target angles
    double r_error = 0.0;

    double l_angle_pid = 0.0; // power diff for angular velocity
    double r_angle_pid = 0.0;

    uint32_t start_t = pros::millis();
    uint32_t elapsed_t = 0;
    bool flag = false;
    bool loop_flag = true;

    //PID instances
    PID left_angle_PID(70, 0, 6000); 
    PID right_angle_PID(70, 0, 6000); 

    vector3D null_v = vector3D(0, 0, 0);

    while (loop_flag)
    {
        target_v = heading;                                                                           // target velocity
        left_angle = wrapAngle(getNormalizedSensorAngle(left_rotation_sensor) - 90.0) * TO_RADIANS;   // takes robot right as 0
        right_angle = wrapAngle(getNormalizedSensorAngle(right_rotation_sensor) - 90.0) * TO_RADIANS; // Y axis positive is front

        current_left_vector = vector3D(cos(left_angle), sin(left_angle), 0.0);  
        current_right_vector = vector3D(cos(right_angle), sin(right_angle), 0.0); 

        v_left = target_v + null_v; //in order to rotate counterclockwise
        v_right = target_v - null_v; 

        bool reverse_right = false;
        bool reverse_left = false;

        // check if the angle is obtuse
        if (v_left * current_left_vector < 0)
        {
            // reverse if angle is obtuse for shorter rotation
            v_left = -v_left;
            reverse_left = true;
        }

        if (v_right * current_right_vector < 0)
        {
            // reverse if angle is obtuse for shorter rotation
            v_right = -v_right;
            reverse_right = true;
        }

        l_error = angle(v_left, current_left_vector);
        r_error = angle(v_right, current_right_vector);

        if (std::isnan(l_error) || std::isnan(r_error))
        {
            l_error = 0.0;
            r_error = 0.0;
        }

        if(fabs(l_error)<(3.0*TO_RADIANS)&&fabs(r_error)<(3.0*TO_RADIANS)){
            if(!flag){
                flag = true;
                start_t = pros::millis();
            }
            elapsed_t = pros::millis() - start_t;
            if (elapsed_t > 200)
            {
                loop_flag = false;
            }
        }

        // angle pid: based on error, pid updates the power to the wheels
        l_angle_pid = left_angle_PID.step(l_error); // power to force anticlockwise aiming
        r_angle_pid = right_angle_PID.step(r_error);

        lu = (int32_t)scale * (l_angle_pid) * 1.0;
        ll = (int32_t)scale * (l_angle_pid) * -1.0;
        ru = (int32_t)scale * (r_angle_pid) * 1.0;
        rl = (int32_t)scale * (-r_angle_pid) * -1.0;

        clampVoltage(MAX_VOLTAGE);
        // limitVoltage(MAX_VOLTAGE);

        std::cout << lu << " : " << ll << " : " << ru << " : " << rl << std::endl;
        move_voltage_wheels(lu, ll, ru, rl);
        pros::delay(2);
    }
    return;
}

void turn180(bool turnleft)
{
    // auton_heading_kP = 0.058; //Without Mogo
    // auton_heading_kI = 0.0;
    // auton_heading_kD = 0.031;
    auton_heading_kP = 0.0525; // Full stack
    auton_heading_kI = 0.0001;
    auton_heading_kD = 0.31;
    double heading = 180.0;
    if (turnleft == true)
        heading *= -1.0;
    moveBaseAutonomous(0.0, 0.0, heading);
}

void turn90(bool turnleft)
{
    // auton_heading_kP = 0.09;
    // auton_heading_kI = 0.0;
    // auton_heading_kD = 0.05;
    auton_heading_kP = 0.15;
    auton_heading_kI = 0.0;
    auton_heading_kD = 0.125;
    double heading = 90.0;
    if (turnleft == true)
        heading *= -1.0;
    moveBaseAutonomous(0.0, 0.0, heading);
}

void turn45(bool turnleft)
{
    auton_heading_kP = 0.3;
    auton_heading_kI = 0.0;
    auton_heading_kD = 0.25;
    // auton_heading_kP = 0.19;
    // auton_heading_kI = 0.0;
    // auton_heading_kD = 20.0;
    double heading = 45.0;
    if (turnleft == true)
        heading *= -1.0;
    moveBaseAutonomous(0.0, 0.0, heading);
}

void turn30(bool turnleft)
{
    // auton_heading_kP = 0.3;
    // auton_heading_kI = 0.0;
    // auton_heading_kD = 0.25;
    auton_heading_kP = 0.19;
    auton_heading_kI = 0.0;
    auton_heading_kD = 20.0;
    double heading = 30.0;
    if (turnleft == true)
        heading *= -1.0;
    moveBaseAutonomous(0.0, 0.0, heading);
}

void turn20(bool turnleft)
{
    // auton_heading_kP = 0.3;
    // auton_heading_kI = 0.0;
    // auton_heading_kD = 0.25;
    auton_heading_kP = 0.19;
    auton_heading_kI = 0.0;
    auton_heading_kD = 20.0;
    double heading = 20.0;
    if (turnleft == true)
        heading *= -1.0;
    moveBaseAutonomous(0.0, 0.0, heading);
}

void mobilegoalopen()
{
    solenoid.set_value(0);
    pros::Task::delay(110);
    mobilegoal_bot.set_value(1);
}

void mobilegoalclose()
{
    solenoid.set_value(1);
    pros::Task::delay(110);
    mobilegoal_bot.set_value(0);
}

void rollerOn()
{
    roller.move(110);
}

void rollerOff()
{
    roller.move(-110);
}

void conveyorOn()
{
    conveyor.move(110);
}

void conveyorOff()
{
    conveyor.move(-110);
}

void conveyorAuton(void* params){
    //100 - 140 hook hues
    //190 - 220 blue hues
    //350 - 10 red hues
    //pros::c::optical_rgb_s_t rgb;
    // if(!tasks_enabled) return;
    int blue = 0;
    int others = 0;
    mobilegoalclose();
    while (true)
    {
        if(pros::millis() > (auton_time + max_auton_time)){
            break;
        }
        conveyor.move(20);
        roller.move(-100);
        double hue_value = conveyor_optical.get_hue();
        // pros::lcd::print(5,"Hue:%lf",hue_value);
        // rgb = conveyor_optical.get_rgb();
        if (!blue_detected)
        {
            if (hue_value > 190.0 && hue_value < 230.0)
            {
                blue_detected = true;
                // pros::lcd::print(3,"blue");
                blue++;
            }
            // else if(!(hue_value > 120.0 && hue_value < 140.0)){
            //     others_detected = true;
            //     others++;
            //     pros::lcd::print(3,"no colour");
            // }
        }
        // pros::lcd::print(1, "blue:%d, others:%d", blue, others);

        if (hue_value > 120.0 && hue_value < 140.0)
        {
            hook_detected = true;
            // pros::lcd::print(3,"hook detected");
        }

            // pros::lcd::print(0,"hue:%.1lf",hue_value);

        if (hook_detected)
        {
            // pros::lcd::print(1,"Hook");
            // ros::delay(200);
            hook_detected = false;
            if (is_we_red_alliance ^ blue_detected)
            {
                conveyor.tare_position();
                conveyor_go_to_score();
                pros::delay(100);
                blue_detected = false;
                // pros::lcd::print(6,"scoring");
            }
            else
            {
                conveyor.tare_position();
                // while(conveyor.get_position() < 555){
                //     conveyor.move(127);
                // }
                // conveyor.move(0);
                // conveyor.brake();
                // pros::delay(50);
                // conveyor.tare_position();
                // while(conveyor.get_position() > -20){
                //     conveyor.move(-127);
                // }
                conveyor_go_to_yeet();
                pros::delay(100);
                // others_detected = false;
                blue_detected = false;
                // pros::lcd::print(6,"yeeting");
            }
            // pros::lcd::print(2,"no Hook");
        }

            pros::delay(2);
    }
}

void yoink(bool yoinketh)
{
    if (yoinketh)
        yoinker.set_value(0);
    else
        yoinker.set_value(1);
}

void positive_blue_auton()
{
    mobilegoalopen();
    rollerOn();
    // score alliance stakes
    moveBaseAutonomous(535.0, 0.0, 0.0);
    conveyor.move(50);
    pros::delay(500);
    rollerOn();
    conveyor.move(0);
    
    // grab mobile goal
    moveBaseAutonomous(-300.0, 0.0, 0.0);
    moveBaseAutonomous(0.0, 200.0, 0.0);
    turn90(true);
    moveBaseAutonomous(0.0, -390.0, 0.0);
    mobilegoalclose();
    moveBaseAutonomous(0.0, -510.0, 0.0);


    // scoring 3 rings and clearing positive corner
    moveBaseAutonomous(-250.0, 0.0, 0.0);
    // tested until here
    //  moveBaseAutonomous(.0,1300.0,0.0);
    moveBaseAutonomous(.0, 650.0, 0.0);
    moveBaseAutonomous(.0, 650.0, 0.0);
    moveBaseAutonomous(.0, -490.0, 0.0);
    moveBaseAutonomous(.0, 490.0, 0.0);


    // scoring 1 more rings and slamming mobile goal into positive corner
    // scoring 1 more rings and slamming mobile goal into positive corner

    moveBaseAutonomous(.0, -700.0, 0.0);
    moveBaseAutonomous(250.0, 0.0, 0.0);
    moveBaseAutonomous(.0, 50.0, 0.0);
    turn180(true);
    moveBaseAutonomous(250.0, 0.0, 0.0);
    moveBaseAutonomous(0.0, -450.0, 0.0);
    mobilegoalopen();
    // ready for match state
    moveBaseAutonomous(.0, 100.0, 0.0);
    moveBaseAutonomous(-500.0, .0, 0.0);

}

void positive_red_auton()
{
    mobilegoalopen();
    rollerOn();
    // score alliance stakes
    moveBaseAutonomous(-535.0, 0.0, 0.0);
    conveyor.move(50);
    pros::delay(500);
    rollerOn();
    conveyor.move(0);

    // grab mobile goal
    moveBaseAutonomous(300.0, 0.0, 0.0);
    moveBaseAutonomous(0.0, 200.0, 0.0);
    turn90(false);  
    moveBaseAutonomous(0.0, -390.0, 0.0);
    mobilegoalclose();
    moveBaseAutonomous(0.0, -510.0, 0.0);

    // scoring 3 rings and clearing positive corner
    moveBaseAutonomous(250.0, 0.0, 0.0);
    // tested until here
    //  moveBaseAutonomous(.0,1300.0,0.0);
    moveBaseAutonomous(.0, 650.0, 0.0);
    moveBaseAutonomous(.0, 650.0, 0.0);
    moveBaseAutonomous(.0, -490.0, 0.0);
    moveBaseAutonomous(.0, 490.0, 0.0);


    // scoring 1 more rings and slamming mobile goal into positive corner


    moveBaseAutonomous(.0, -700.0, 0.0);
    moveBaseAutonomous(-250.0, 0.0, 0.0);
    moveBaseAutonomous(.0, 50.0, 0.0);
    turn180(false);
    moveBaseAutonomous(-250.0, 0.0, 0.0);
    moveBaseAutonomous(0.0, 450.0, 0.0);
    mobilegoalopen();
    // ready for match state
    moveBaseAutonomous(.0, 100.0, 0.0);
    moveBaseAutonomous(500.0, .0, 0.0);
    
}

void negative_blue_auton()
{
    mobilegoalopen();
    yoinker_actuated = !yoinker_actuated;
    yoink(yoinker_actuated);

    moveBaseAutonomous(0.0, -750.0, 0.0);
    moveBaseAutonomous(300.0, 0.0, 0.0);

    moveBaseAutonomous(0.0, -60.0, 0.0);
    mobilegoalclose();
    // start scoring thread
    rollerOn();
    // conveyorOn();
    moveBaseAutonomous(0.0, 900.0, 0.0);
    moveBaseAutonomous(0.0, 0.0, -49.7);
    moveBaseAutonomous(0.0, 413.0, 0.0);

    moveBaseAutonomous(0.0, -150.0, 0.0);
    moveBaseAutonomous(0.0, 150.0, 0.0);
    moveBaseAutonomous(0.0, -300.0, 0.0);
    rollerOff();
    //conveyorOff();
    slammingState = SLAM_LADDER;
    moveBaseAutonomous(0.0, -1300.0, 0.0);
}

void negative_red_auton()
{
    mobilegoalopen();
    yoinker_actuated = !yoinker_actuated;
    yoink(yoinker_actuated);

    moveBaseAutonomous(0.0, -750.0, 0.0);
    moveBaseAutonomous(-300.0, 0.0, 0.0);

    moveBaseAutonomous(0.0, -60.0, 0.0);
    mobilegoalclose();
    // start scoring thread
    rollerOn();
    // conveyorOn();
    moveBaseAutonomous(0.0, 900.0, 0.0);
    moveBaseAutonomous(0.0, 0.0, 49.7);
    moveBaseAutonomous(0.0, 413.0, 0.0);

    moveBaseAutonomous(0.0, -150.0, 0.0);
    moveBaseAutonomous(0.0, 150.0, 0.0);
    moveBaseAutonomous(0.0, -300.0, 0.0);
    rollerOff();
    conveyorOff();
    slammingState = SLAM_LADDER;
    moveBaseAutonomous(0.0, -1300.0, 0.0);
}

void test()
{
    vector3D targetheading (0,MAX_SPEED,0);
    // pros::lcd::print(2,"test bef align wheel");
    // alignWheels(targetheading);
    pros::lcd::print(2,"test after align wheel");
    mobilegoalclose();
    moveBaseAutonomous(0.0, -60.0, 0.0);
    mobilegoalopen();
    // pros::delay(3000);
    //      

    //      alignWheels(targetheading);
    //      
    //     moveBaseAutonomous(0.0, 60.0, 0.0);
    //     pros::delay(3000);
    //     // alignWheels(targetheading);
    //     moveBaseAutonomous(0.0, -150.0, 0.0);
    //     pros::delay(3000);
    //     // alignWheels(targetheading);
    //     moveBaseAutonomous(0.0, 150.0, 0.0);
    //     pros::delay(3000);
    //     // alignWheels(targetheading);
    // moveBaseAutonomous(0.0, -200.0, 0.0);
    // pros::delay(3000);
    // alignWheels(targetheading);
    moveBaseAutonomous(0.0, 700.0, 0.0);
    moveBaseAutonomous(0.0, 700.0, 0.0);

    pros::delay(3000);
}

void competition_initialize() {

}

void disabled() {
    // tasks_enabled = false;
    // serial_task.suspend();
    // conveyor_auton.suspend();
}

void autonomous()
{
    // serial_read = pros::Task serial_task(serialRead, (void*)"serial", TASK_PRIORITY_DEFAULT,
    //                 TASK_STACK_DEPTH_DEFAULT, "Serial read task");
    // tasks_enabled = true;
    // serial_task.resume();
    // conveyor_auton.resume();
    pros::Task conveyor_auton(conveyorAuton, (void *)"conveyor", TASK_PRIORITY_DEFAULT,
                        TASK_STACK_DEPTH_DEFAULT, "conveyor auton");
    auton_time = pros::millis();
    positive_blue_auton();
    is_we_red_alliance = false;
    // conveyor_auton.suspend();
    // conveyor_auton.remove();

}

// pros::Task serial_task(serialRead, (void *)"serial", TASK_PRIORITY_DEFAULT + 1,
//                        TASK_STACK_DEPTH_DEFAULT, "serial task");

void initialize()
{
    pros::lcd::initialize();
    conveyor_optical.set_led_pwm(100);
    conveyor_optical.set_integration_time(2);
    tasks_enabled = true;
    pros::Task serial_task(serialRead, (void*)"serial", TASK_PRIORITY_DEFAULT,
                    TASK_STACK_DEPTH_DEFAULT, "Serial read task");
    // if (serial_task_enabled == false)
    // { // Test code, remove for actual match code
    //     pros::Task serial_task(serialRead, (void *)"serial", TASK_PRIORITY_DEFAULT,
    //                            TASK_STACK_DEPTH_DEFAULT, "serial task");
    //     serial_task_enabled = true;
    //     pros::delay(15);
    // }
    // //while(!imu.reset(true)&&!imu2.reset(true));
    // vexGenericSerialEnable(SERIALPORT - 1, 0);
    // vexGenericSerialBaudrate(SERIALPORT - 1, 115200);
    // pros::Task serial_task(serialRead, (void*)"serial", TASK_PRIORITY_DEFAULT,
    //                 TASK_STACK_DEPTH_DEFAULT, "Serial read task");
    // while(!tasks_enabled) pros::delay(1);
    // pros::Task conveyor_auton(conveyorAuton, (void *)"conveyor", TASK_PRIORITY_DEFAULT,
    //                 TASK_STACK_DEPTH_DEFAULT, "conveyor auton");
    pros::delay(10);

    while (imu.reset(true) == PROS_ERR)
        ;
    while (imu.set_data_rate(5) == PROS_ERR)
        ;

    pros::delay(15); // IMU Calibrated

    setBrakeModes();
    // setMotorCurrentLimit(MAX_CURRENT_BASE);

    while (left_rotation_sensor.set_data_rate(5) == PROS_ERR)
        ;
    while (right_rotation_sensor.set_data_rate(5) == PROS_ERR)
        ;

    // left_rotation_sensor.set_position(0);
    // right_rotation_sensor.set_position(0);

    // imu.reset(true);  //uncomment for actual
    // pros::delay(100);
    // master.print(0,0,"IMU calibrated  ");
    while (global_distX == 0.0 || global_distY == 0.0)
    {
    }
    // imu2.set_data_rate(5);
    pros::Task slam_dunk(slamDunk, (void *)"slam", TASK_PRIORITY_DEFAULT,
                         TASK_STACK_DEPTH_DEFAULT, "slam task");
    // pros::Task conveyor_auton(conveyorAuton, (void *)"conveyor", TASK_PRIORITY_DEFAULT + 1,
    //                      TASK_STACK_DEPTH_DEFAULT, "conveyor auton");
    master.rumble(".-.");
}

void opcontrol(){   //TODO: JOEL PLEASE MAKE CONVEYOR A TASK
    //serial_task.remove();
    tasks_enabled = false;
    pros::Task move_base(moveBase, (void*)"driver", TASK_PRIORITY_MAX-2,
                    TASK_STACK_DEPTH_DEFAULT, "driver task");
    while (true)
    {
        if(driver == true) move_base.resume();
        else move_base.suspend();

        leftY = master.get_analog(ANALOG_LEFT_Y);

        if (arcade == true)
            leftX = 0.0;
        else
            leftX = master.get_analog(ANALOG_LEFT_X);

        rightX = master.get_analog(ANALOG_RIGHT_X);
        rightY = master.get_analog(ANALOG_RIGHT_Y);

        if (master.get_digital_new_press(DIGITAL_A))
            mobile_goal_actuated = !mobile_goal_actuated;
        if (master.get_digital_new_press(DIGITAL_B))
            brake();
        if (master.get_digital_new_press(DIGITAL_X))
            slam_dunk_actuated = !slam_dunk_actuated;

        // if(master.get_digital(DIGITAL_B)) brake();

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
            conveyor.move(110);
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
            conveyor.move(-110);
        else
            conveyor.move(0);

        // L1 FORWARD, L2 BACKWARD FOR ROLLER (missing hardware)
        // when L1 is pressed, rollers move forward with NEGATIVE velocity??
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
            roller.move(110);
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
            roller.move(-110);
        else
            roller.move(0);

                if (mobile_goal_actuated)
                {
                    solenoid.set_value(1);
                    pros::Task::delay(110);
                    mobilegoal_bot.set_value(0);
                }
                else
                {
                    solenoid.set_value(0);
                    pros::Task::delay(110);
                    mobilegoal_bot.set_value(1);
                }

        if (slam_dunk_actuated)
            slam_in_out.set_value(1);
        else
            slam_in_out.set_value(0);

        if (master.get_digital_new_press(DIGITAL_UP))
        {
            slammingState = SLAM_EXTENDED_STATE;
        }
        else if (master.get_digital_new_press(DIGITAL_RIGHT))
        {
            slammingState = SLAM_MID_STATE;
        }
        else if (master.get_digital_new_press(DIGITAL_DOWN))
        {
            slammingState = SLAM_START_STATE;
        }

        // if(master.get_digital_new_press(DIGITAL_LEFT)) arcade = !arcade;
        // if(master.get_digital_new_press(DIGITAL_LEFT)) driver = !driver;
        if (master.get_digital_new_press(DIGITAL_LEFT))
            yoinker_actuated = !yoinker_actuated;
        if (master.get_digital_new_press(DIGITAL_Y))
            roller_lifts = !roller_lifts;

        yoink(yoinker_actuated);

        if (roller_lifts)
        {
            roller_lifter.set_value(1);
        }
        else
        {
            roller_lifter.set_value(0);
        }

        pros::delay(2);
    }
}