#include "main.h"
#include "pros/misc.h"
#include "pros/motors.h"

void disabled() {}
void competition_initialize() {}

// Function to determine sign of a integer variable, returns bool
template <typename T>
int sgn(T val) { return (T(0) < val) - (val < T(0)); }

void serialRead(void *params)
{
    vexGenericSerialEnable(SERIALPORT - 1, 0);
    vexGenericSerialBaudrate(SERIALPORT - 1, 115200);
    pros::delay(10);
    pros::screen::set_pen(COLOR_BLUE);
    double distX, distY = 0;
    while (true)
    {
        uint8_t buffer[256];
        int bufLength = 256;
        int32_t nRead = vexGenericSerialReceive(SERIALPORT - 1, buffer, bufLength);
        if (nRead >= 0)
        {
            std::stringstream dataStream("");
            bool recordOpticalX, recordOpticalY = false;
            for (int i = 0; i < nRead; i++)
            {
                char thisDigit = (char)buffer[i];
                if (thisDigit == 'D' || thisDigit == 'I' || thisDigit == 'A' ||
                    thisDigit == 'X' || thisDigit == 'C' || thisDigit == 'Y')
                {
                    recordOpticalX = false;
                    recordOpticalY = false;
                }
                if (thisDigit == 'C')
                {
                    recordOpticalX = false;
                    dataStream >> distX;
                    // pros::lcd::print(1, "Optical Flow:");
                    global_distX = distX * 10;
                    pros::lcd::print(2, "distX: %.2lf", distX * 10);
                    dataStream.str(std::string());
                }
                if (thisDigit == 'D')
                {
                    recordOpticalY = false;
                    dataStream >> distY;
                    global_distY = distY * 10;
                    pros::lcd::print(3, "distY: %.2lf", global_distY);
                    dataStream.str(std::string());
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

double wrapAngle(double angle)
{ // forces the angle to be within the -180 <
    // angle < 180 range
    if (angle > 180.0)
        while (angle > 180.0)
            angle -= 360.0;
    else if (angle < -180.0)
        while (angle < -180.0)
            angle += 360.0;
    return angle;
}

double getNormalizedSensorAngle(
    pros::Rotation &sensor)
{ // Converts rotational sensor readings into
    // degrees and bounds it between -180 to 180
    double angle =
        sensor.get_angle() / 100.0; // Convert from centidegrees to degrees
    return wrapAngle(
        angle); // forces the angle to be within the -180 < angle < 180 range
}

void set_wheel_angle(float target_angle, double kP_set_wheel = 2.0,
                     double kI_set_wheel = 0.00, double kD_set_wheel = 0.2)
{
    float left_current_angle = getNormalizedSensorAngle(left_rotation_sensor);
    float right_current_angle = getNormalizedSensorAngle(right_rotation_sensor);

    float left_error = target_angle - left_current_angle;
    float right_error = target_angle - right_current_angle;

    float left_previous_error = 0;
    float right_previous_error = 0;

    float left_integral = 0;
    float right_integral = 0;

    // Normalize the errors to be within [-180, 180] degrees
    while (left_error > 180)
        left_error -= 360;
    while (left_error < -180)
        left_error += 360;
    while (right_error > 180)
        right_error -= 360;
    while (right_error < -180)
        right_error += 360;

    int max_attempts = 50; // Set a maximum number of attempts to adjust the angle
    int attempts = 0;

    while ((fabs(left_error) > 2 || fabs(right_error) > 2) &&
           attempts < max_attempts)
    {
        left_current_angle = getNormalizedSensorAngle(left_rotation_sensor);
        right_current_angle = getNormalizedSensorAngle(right_rotation_sensor);

        left_error = target_angle - left_current_angle;
        right_error = target_angle - right_current_angle;

        while (left_error > 180)
            left_error -= 360;
        while (left_error < -180)
            left_error += 360;
        while (right_error > 180)
            right_error -= 360;
        while (right_error < -180)
            right_error += 360;

        left_integral += left_error;
        right_integral += right_error;

        float left_derivative = left_error - left_previous_error;
        float right_derivative = right_error - right_previous_error;

        left_previous_error = left_error;
        right_previous_error = right_error;

        float left_motor_speed = kP_set_wheel * left_error + kD_set_wheel * left_derivative;
        float right_motor_speed = kP_set_wheel * right_error + kD_set_wheel * right_derivative;
        pros::lcd::print(0, "inside setwheel");

        luA.move(left_motor_speed);
        luB.move(left_motor_speed);
        llA.move(-left_motor_speed);
        llB.move(-left_motor_speed);

        ruA.move(right_motor_speed);
        ruB.move(right_motor_speed);
        rlA.move(-right_motor_speed);
        rlB.move(-right_motor_speed);

        pros::delay(2); // Refresh rate within the set_wheel_angle function
        attempts++;
    }

    // Stop the motors
    luA.move(0);
    luB.move(0);
    llA.move(0);
    llB.move(0);
    ruA.move(0);
    ruB.move(0);
    rlA.move(0);
    rlB.move(0);
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

void clampVoltage(double lu, double ll, double ru, double rl)
{
    // if any of lu, ll, ru or rl are too big, we need to scale them, and we must
    // scale them all by the same amount so we dont throw off the proportions
    if (fabs(lu) > MAX_VOLTAGE || fabs(ll) > MAX_VOLTAGE ||
        fabs(ru) > MAX_VOLTAGE || fabs(rl) > MAX_VOLTAGE)
    {
        // figure out which of lu, ll, ru or rl has the largest magnitude
        double max = fabs(lu);
        if (max < fabs(ll))
            max = fabs(ll);
        if (max < fabs(ru))
            max = fabs(ru);
        if (max < fabs(rl))
            max = fabs(rl);
        double VoltageScalingFactor =
            max / MAX_VOLTAGE; // this will definitely be positive, hence it wont
                               // change the sign of lu, ll, ru or rl.
        lu = lu / VoltageScalingFactor;
        ll = ll / VoltageScalingFactor;
        ru = ru / VoltageScalingFactor;
        rl = rl / VoltageScalingFactor;
    }
}

double angle(vector3D v1, vector3D v2)
{
    double dot = v1 * v2;
    double det = v1.x * v2.y - v1.y * v2.x;
    return -atan2(det, dot); // atan2 automatically considers the sign to deal
                             // with the trigonometry quadrant
}

void turn_angle(double targetTurning, double turn_Kp, double turn_Kd)
{
    while (!imu.tare_rotation())
        ;
    double initialHeading = imu.get_rotation(); // Store the initial heading
    double currentHeading = initialHeading;
    double turnError = 0.0;

    // 1. Perform turning first

    double prevError = 0.0;
    double l_angleMaintain =
        (0 - 90.0) *
        TO_RADIANS; // note that the function getNormalizedSensorAngle already
                    // implements wrapAngle to bound the angle between -180 and
                    // 180 degrees
    double r_angleMaintain =
        (0 - 90.0) *
        TO_RADIANS; // note that the function getNormalizedSensorAngle already
                    // implements wrapAngle to bound the angle between -180 and
                    // 180 degrees

    double left_angle, right_angle;
    double l_error = 0.0;
    double r_error = 0.0;
    // power output for angle component of pid
    double l_angle_pid = 0.0;
    double r_angle_pid = 0.0;
    // 1. Perform turning first

    PID left_angle_PID(angle_kP, angle_kI, angle_kD);
    PID right_angle_PID(angle_kP, angle_kI, angle_kD);
    while (true)
    {
        left_angle = (getNormalizedSensorAngle(left_rotation_sensor) - 90.0) *
                     TO_RADIANS; // note that the function getNormalizedSensorAngle
                                 // already implements wrapAngle to bound the angle
                                 // between -180 and 180 degrees
        right_angle =
            (getNormalizedSensorAngle(right_rotation_sensor) - 90.0) * TO_RADIANS;

        vector3D l_target_angle =
            vector3D(cos(l_angleMaintain), sin(l_angleMaintain), 0);
        vector3D r_target_angle =
            vector3D(cos(r_angleMaintain), sin(r_angleMaintain), 0);
        vector3D l_current_angle = vector3D(cos(left_angle), sin(left_angle), 0);
        vector3D r_current_angle = vector3D(cos(right_angle), sin(right_angle), 0);

        l_error = angle(l_current_angle, l_target_angle);
        r_error = angle(r_current_angle, r_target_angle);

        // calculate the PID output
        l_angle_pid = left_angle_PID.step(l_error);
        r_angle_pid = right_angle_PID.step(r_error);
        currentHeading = fabs(imu.get_rotation());
        turnError = fabs(fabs(targetTurning) - (currentHeading));
        pros::lcd::print(0, "Error: %.lf", turnError);

        // Check if we are within the error threshold
        if (fabs(turnError) <= 1.0)
        {
            // Stop turning if we are close enough
            brake();
            luA.move(0); // Adjust left side for turning
            luB.move(0);
            llA.move(0);
            llA.move(0);
            ruA.move(0); // Adjust right side for turning
            ruB.move(0);
            rlA.move(0);
            rlB.move(0);
            break; // Exit turning loop
        }
        double turnDerivative = prevError - turnError;

        // Calculate turn power
        double turnPower =
            turnError * turn_Kp + turnDerivative * turn_Kd; // Tune this gain
        prevError = turnError;

        // Adjust motor powers for turning
        if (targetTurning > 0)
        {
            luA.move(turnPower - l_angle_pid); // Adjust left side for turning
            luB.move(turnPower - l_angle_pid);
            llA.move(turnPower + l_angle_pid);
            llA.move(turnPower + l_angle_pid);
            ruA.move(-turnPower - r_angle_pid); // Adjust right side for turning
            ruB.move(-turnPower - r_angle_pid);
            rlA.move(-turnPower + r_angle_pid);
            rlB.move(-turnPower + r_angle_pid);
        }
        else if (targetTurning < 0)
        {
            luA.move(-turnPower - l_angle_pid); // Adjust left side for turning
            luB.move(-turnPower - l_angle_pid);
            llA.move(-turnPower + l_angle_pid);
            llA.move(-turnPower + l_angle_pid);
            ruA.move(turnPower - r_angle_pid); // Adjust right side for turning
            ruB.move(turnPower - r_angle_pid);
            rlA.move(turnPower + r_angle_pid);
            rlB.move(turnPower + r_angle_pid);
        }

        if (fabs(turnError) > fabs(targetTurning))
        {
            brake();
            luA.move(0); // Adjust left side for turning
            luB.move(0);
            llA.move(0);
            llA.move(0);
            ruA.move(0); // Adjust right side for turning
            ruB.move(0);
            rlA.move(0);
            rlB.move(0);
            break;
        }

        pros::delay(5); // Delay to reduce CPU load
    }
}

void update_turning(double turn_target_angle, double kP, double kD)
{

    is_turning = true;
    turn_previous_error = 0;
    double powerL = 0;
    double powerR = 0;
    double errorLeft = 0;
    double errorRight = 0;
    double prevErrorLeft = 0;
    double prevErrorRight = 0;
    double totalErrorLeft = 0;
    double totalErrorRight = 0;

    double offset_error = 0;
    double prev_offset_error = 0;
    double offset_correction = 0;

    double l_angleMaintain =
        (0 - 90.0) *
        TO_RADIANS; // note that the function getNormalizedSensorAngle already
                    // implements wrapAngle to bound the angle between -180 and
                    // 180 degrees
    double r_angleMaintain =
        (0 - 90.0) *
        TO_RADIANS; // note that the function getNormalizedSensorAngle already
                    // implements wrapAngle to bound the angle between -180 and
                    // 180 degrees

    double left_angle, right_angle;
    double l_error = 0.0;
    double r_error = 0.0;
    // power output for angle component of pid
    double l_angle_pid = 0.0;
    double r_angle_pid = 0.0;
    // 1. Perform turning first

    PID left_angle_PID(angle_kP, angle_kI, angle_kD);
    PID right_angle_PID(angle_kP, angle_kI, angle_kD);
    double current_angle = imu.get_heading();

    double error = turn_target_angle - current_angle;

    while (fabs(error) > 2)
    {
        left_angle = (getNormalizedSensorAngle(left_rotation_sensor) - 90.0) *
                     TO_RADIANS; // note that the function getNormalizedSensorAngle
                                 // already implements wrapAngle to bound the angle
                                 // between -180 and 180 degrees
        right_angle =
            (getNormalizedSensorAngle(right_rotation_sensor) - 90.0) * TO_RADIANS;

        vector3D l_target_angle =
            vector3D(cos(l_angleMaintain), sin(l_angleMaintain), 0);
        vector3D r_target_angle =
            vector3D(cos(r_angleMaintain), sin(r_angleMaintain), 0);
        vector3D l_current_angle = vector3D(cos(left_angle), sin(left_angle), 0);
        vector3D r_current_angle = vector3D(cos(right_angle), sin(right_angle), 0);

        l_error = angle(l_current_angle, l_target_angle);
        r_error = angle(r_current_angle, r_target_angle);

        // calculate the PID output
        l_angle_pid = left_angle_PID.step(l_error);
        r_angle_pid = right_angle_PID.step(r_error);
        current_angle = imu.get_heading();
        error = turn_target_angle - current_angle;
        double derivative = error - turn_previous_error;
        turn_previous_error = error;

        double power = kP * error + kD * derivative;
        pros::lcd::print(2, "Current Angle: %f", current_angle);
        pros::lcd::print(3, "Error: %f", error);
        pros::lcd::print(4, "Power: %f", power);

        luA.move(power + l_angle_pid); // Adjust left side for turning
        luB.move(power + l_angle_pid);
        llA.move(power - l_angle_pid);
        llA.move(power - l_angle_pid);
        ruA.move(-power + r_angle_pid); // Adjust right side for turning
        ruB.move(-power + r_angle_pid);
        rlA.move(-power - r_angle_pid);
        rlB.move(-power - r_angle_pid);
        pros::delay(5);
        if (fabs(error) < 2)
        {

            brake();
            luA.move(0); // Adjust left side for turning
            luB.move(0);
            llA.move(0);
            llA.move(0);
            ruA.move(0); // Adjust right side for turning
            ruB.move(0);
            rlA.move(0);
            rlB.move(0);
            break;
        }
    }
}

void set_wheel_angle_new(float targetangle, double kP_set_wheel = 2.0,
                         double kI_set_wheel = 0.00, double kD_set_wheel = 0.2)
{

    double l_angleMaintain =
        (targetangle - 90.0) *
        TO_RADIANS; // note that the function getNormalizedSensorAngle already
                    // implements wrapAngle to bound the angle between -180 and
                    // 180 degrees
    double r_angleMaintain =
        (targetangle - 90.0) *
        TO_RADIANS; // note that the function getNormalizedSensorAngle already
                    // implements wrapAngle to bound the angle between -180 and
                    // 180 degrees
    float left_current_angle = getNormalizedSensorAngle(left_rotation_sensor);
    float right_current_angle = getNormalizedSensorAngle(right_rotation_sensor);

    double left_angle, right_angle;
    double l_error = 0.0;
    double r_error = 0.0;
    // power output for angle component of pid
    double l_angle_pid = 0.0;
    double r_angle_pid = 0.0;
    PID left_angle_PID(kP_set_wheel, kI_set_wheel, kD_set_wheel);
    PID right_angle_PID(kP_set_wheel, kI_set_wheel, kD_set_wheel);

    int max_attempts = 50; // Set a maximum number of attempts to adjust the angle
    int attempts = 0;
    float left_error = target_angle - left_current_angle;
    float right_error = target_angle - right_current_angle;

    while ((fabs(left_error) > 2 || fabs(right_error) > 2) &&
           attempts < max_attempts)
    {
        left_angle = (getNormalizedSensorAngle(left_rotation_sensor) - 90.0) *
                     TO_RADIANS; // note that the function getNormalizedSensorAngle
                                 // already implements wrapAngle to bound the angle
                                 // between -180 and 180 degrees
        right_angle =
            (getNormalizedSensorAngle(right_rotation_sensor) - 90.0) * TO_RADIANS;

        vector3D l_target_angle =
            vector3D(cos(l_angleMaintain), sin(l_angleMaintain), 0);
        vector3D r_target_angle =
            vector3D(cos(r_angleMaintain), sin(r_angleMaintain), 0);
        vector3D l_current_angle = vector3D(cos(left_angle), sin(left_angle), 0);
        vector3D r_current_angle = vector3D(cos(right_angle), sin(right_angle), 0);

        l_error = angle(l_current_angle, l_target_angle);
        r_error = angle(r_current_angle, r_target_angle);

        // calculate the PID output
        l_angle_pid = left_angle_PID.step(l_error);
        r_angle_pid = right_angle_PID.step(r_error);

        pros::lcd::print(1, "inside setwheel");

        luA.move(l_angle_pid);
        luB.move(l_angle_pid);
        llA.move(-l_angle_pid);
        llB.move(-l_angle_pid);

        ruA.move(r_angle_pid);
        ruB.move(r_angle_pid);
        rlA.move(-r_angle_pid);
        rlB.move(-r_angle_pid);

        pros::delay(2); // Refresh rate within the set_wheel_angle function
        attempts++;
    }

    // Stop the motors
    luA.move(0);
    luB.move(0);
    llA.move(0);
    llB.move(0);
    ruA.move(0);
    ruB.move(0);
    rlA.move(0);
    rlB.move(0);
}

void base_PID_front_back(double base_kp, double base_ki, double base_kd,
                         double targetangle, double targetDistance_Y,
                         double decelerationThreshold, double offset_kp, double offset_kd)
{
    // Movement variables
    double powerL = 0;
    double powerR = 0;
    double errorLeft = 0;
    double errorRight = 0;
    double prevErrorLeft = 0;
    double prevErrorRight = 0;
    double totalErrorLeft = 0;
    double totalErrorRight = 0;

    double offset_error = 0;
    double prev_offset_error = 0;
    double offset_correction = 0;

    double l_angleMaintain =
        (targetangle - 90.0) *
        TO_RADIANS; // note that the function getNormalizedSensorAngle already
                    // implements wrapAngle to bound the angle between -180 and
                    // 180 degrees
    double r_angleMaintain =
        (targetangle - 90.0) *
        TO_RADIANS; // note that the function getNormalizedSensorAngle already
                    // implements wrapAngle to bound the angle between -180 and
                    // 180 degrees

    double left_angle, right_angle;
    double l_error = 0.0;
    double r_error = 0.0;
    // power output for angle component of pid
    double l_angle_pid = 0.0;
    double r_angle_pid = 0.0;
    // 1. Perform turning first

    bool l_move = true;
    bool r_move = true;
    int timeout = 0;

    PID left_angle_PID(angle_kP, angle_kI, angle_kD);
    PID right_angle_PID(angle_kP, angle_kI, angle_kD);

    double aim_target = targetDistance_Y - global_distY;
    original_x = global_distX;

    vector3D imu_angular;
    vector3D angular_error;
    vector3D rot_pid;
    vector3D rot_FF;

    double rot_vector_double = 0.0;
    double rot_pid_double = 0.0;
    double gyro_rate = 0.0;
    double a_err_d = 0.0;
    double current_angular = 0.0;
    PID rotate_robot_PID(azim_kP, azim_kI, azim_kD);
    vector3D L2I_pos(WHEEL_BASE_RADIUS, 0.0, 0.0);
    vector3D current_left_vector; // direction unit vector for wheels
    vector3D current_right_vector;

    while (l_move || r_move)
    {

        if (imu.is_calibrating())
        {
            gyro_rate = current_angular; // ignore gyro while calibrating, use encoder values
        }
        else
        {
            gyro_rate = -1.0 * imu.get_gyro_rate().z * TO_RADIANS;
        }

        imu_angular = vector3D(0.0, 0.0, gyro_rate); // Radians per second, loaded as angle

        angular_error = target_r - imu_angular;
        if (fabs(angular_error.z) < ANGULAR_THRESH)
        {
            angular_error.load(0.0, 0.0, 0.0);
            rot_pid_double = 0.0;
        }

        a_err_d = angular_error.getZ();
        rot_pid_double += rotate_robot_PID.step(a_err_d);
        // pros::lcd::print(3, "imu_angular %3.8f", imu_angular.z);
        // pros::lcd::print(6,"rot_v_d %3.8f", rot_vector_double);
        rot_FF = (target_r ^ L2I_pos).scalar(r_kF_STATIC);
        rot_vector_double = rot_pid_double + rot_FF.getY();
        rot_pid = vector3D(0.0, rot_vector_double, 0.0);

        left_angle = wrapAngle(getNormalizedSensorAngle(left_rotation_sensor) - 90.0) * TO_RADIANS;   // takes robot right as 0
        right_angle = wrapAngle(getNormalizedSensorAngle(right_rotation_sensor) - 90.0) * TO_RADIANS; // Y axis positive is front

        current_left_vector = vector3D(cos(left_angle), sin(left_angle), 0.0);
        current_right_vector = vector3D(cos(right_angle), sin(right_angle), 0.0);

        // vector3D l_target_angle =
        //     vector3D(cos(l_angleMaintain), sin(l_angleMaintain), 0);
        // vector3D r_target_angle =
        //     vector3D(cos(r_angleMaintain), sin(r_angleMaintain), 0);
        // vector3D l_current_angle = vector3D(cos(left_angle), sin(left_angle), 0);
        // vector3D r_current_angle = vector3D(cos(right_angle), sin(right_angle), 0);

        l_error = angle(v_left, current_left_vector);
        r_error = angle(v_right, current_right_vector);

        // calculate the PID output
        l_angle_pid = left_angle_PID.step(l_error);
        r_angle_pid = right_angle_PID.step(r_error);
        // Calculate distance error

        // double targetDistance = sqrt((pow ( fabs(targetDistance_X) -
        // fabs(global_distX),2) +  pow (fabs(targetDistance_Y) -
        // fabs(global_distY),2) ));
        double targetDistance = aim_target - global_distY;
        pros::lcd::print(
            4, "current_distance: %.lf",
            targetDistance);

        errorLeft = targetDistance;
        errorRight = targetDistance;
        totalErrorLeft += errorLeft;
        totalErrorRight += errorRight;

        offset_error = global_distX - original_x;
        float offset_derivative = offset_error - prev_offset_error;
        prev_offset_error = offset_error;
        offset_correction = (offset_kp * offset_error) + (offset_kd * offset_derivative);

        // PID for left motors
        if (fabs(errorLeft) <= base_error)
        {
            powerL = 0.0;
            l_move = false;
        }
        else
        {
            // Gradually reduce power as you approach the target
            if (fabs(errorLeft) < decelerationThreshold)
            {
                powerL *= 0.5; // Reduce power to half when close to target
            }
            else
            {
                powerL = base_kp * errorLeft + base_ki * totalErrorLeft +
                         base_kd * (errorLeft - prevErrorLeft);
            }
            powerL = std::clamp(powerL, -540.0, 540.0);
        }

        // PID for right motors
        if (fabs(errorRight) <= base_error)
        {
            powerR = 0.0;
            r_move = false;
        }
        else
        {
            // Gradually reduce power as you approach the target
            if (fabs(errorRight) < decelerationThreshold)
            {
                powerR *= 0.5; // Reduce power to half when close to target
            }
            else
            {
                powerR = base_kp * errorRight + base_ki * totalErrorRight +
                         base_kd * (errorRight - prevErrorRight);
            }
            powerR = std::clamp(powerR, -540.0, 540.0);
        }
        pros::lcd::print(
            5, "powerL: %.lf",
            powerL);
        pros::lcd::print(
            6, "powerR: %.lf",
            powerR);
        // Move the motors
        if (targetDistance_Y >= 0)
        {
            pros::lcd::print(
                0, "targetDistance_Y: %.lf",
                targetDistance_Y);
            luA.move_velocity(powerL - l_angle_pid);
            luB.move_velocity(powerL - l_angle_pid);
            llA.move_velocity(powerL + l_angle_pid);
            llB.move_velocity(powerL + l_angle_pid);
            ruA.move_velocity(powerR - r_angle_pid);
            ruB.move_velocity(powerR - r_angle_pid);
            rlA.move_velocity(powerR + r_angle_pid);
            rlB.move_velocity(powerR + r_angle_pid);
            if (errorRight < 0 && errorLeft < 0)
            {
                brake();
                break;
            }
        }
        else
        {
            pros::lcd::print(
                0, "targetDistance_Y: %.lf",
                targetDistance_Y);
            luA.move_velocity(powerL - l_angle_pid);
            luB.move_velocity(powerL - l_angle_pid);
            llA.move_velocity(powerL + l_angle_pid);
            llB.move_velocity(powerL + l_angle_pid);
            ruA.move_velocity(powerR - r_angle_pid);
            ruB.move_velocity(powerR - r_angle_pid);
            rlA.move_velocity(powerR + r_angle_pid);
            rlB.move_velocity(powerR + r_angle_pid);
            if (errorRight > 0 && errorLeft > 0)
            {
                brake();
                break;
            }
        }

        if (timeout >= 100000)
            break;

        if (fabs(powerR) < 2 || fabs(powerL) < 2)
        {
            break;
        }

        // pros::lcd::print(0, "ErrorL: %.lf", errorLeft);
        // pros::lcd::print(1, "ErrorR: %.lf", errorRight);

        // Update previous errors for the next iteration
        prevErrorLeft = errorLeft;
        prevErrorRight = errorRight;

        pros::delay(2); // Delay to reduce CPU load
        timeout += 2;
    }
    brake();
}

// void base_PID_front_back_imu_x(double base_kp, double base_kd, double offset_kp, double offset_kd,
//                                double targetangle, double targetDistance_Y,
//                                double decelerationThreshold, double angle_offset_p, double angle_offset_kd)
// {
//     // Movement variables
//     double power = 0;
//     double errordistance = 0;

//     double totalErrordistance = 0;
//     double offset_error = 0;
//     double prev_offset_error = 0;
//     double offset_correction = 0;

// double angleMaintain =
//     (targetangle - 90.0) *
//     TO_RADIANS; // note that the function getNormalizedSensorAngle already
//                 // implements wrapAngle to bound the angle between -180 and
//                 // 180 degrees

//     // power output for angle component of pid
//     double angle_pid = 0.0;

//     bool move = true;
//     int timeout = 0;

//     PID angle_PID(angle_offset_p, angle_kI, angle_offset_kd);

//     double aim_target = targetDistance_Y - global_distY;
//     original_x = global_distX;
//     double heading_error;
//     double prev_errordistance;

//     while (move)
//     {
//         pros::lcd::print(
//             1, "moving"
//             );

//         imu_angle = (wrapAngle(imu.get_heading() - 90.0) *
//                      TO_RADIANS); // note that the function getNormalizedSensorAngle
//                                   // already implements wrapAngle to bound the angle
//                                   // between -180 and 180 degrees

// vector3D imu_target_angle_vector =
//     vector3D(cos(angleMaintain), sin(angleMaintain), 0);

// vector3D current_angle = vector3D(cos(imu_angle), sin(imu_angle), 0);

//         heading_error = angle(l_current_angle, target_angle_vector);

//         // calculate the PID output
//         angle_pid = angle_PID.step(heading_error);

//         double targetDistance = aim_target - global_distY;
//         pros::lcd::print(
//             4, "current_distance: %.lf",
//             targetDistance);

//         errordistance = targetDistance;
//         float errordistance_derivative = errordistance - prev_errordistance;
//         prev_errordistance = errordistance;

//         offset_error = global_distX - original_x;
//         float offset_derivative = offset_error - prev_offset_error;
//         prev_offset_error = offset_error;
//         offset_correction = (offset_kp * offset_error) + (offset_kd * offset_derivative);

//         // PID for left motors
//         if (fabs(errordistance) <= base_error)
//         {
//             power = 0.0;
//             move = false;
//         }
//         else
//         {
//             // Gradually reduce power as you approach the target
//             if (fabs(errordistance) < decelerationThreshold)
//             {
//                 power *= 0.5; // Reduce power to half when close to target
//             }
//             else
//             {
//                 power = base_kp * errordistance +
//                         base_kd * errordistance_derivative;
//             }
//             power = std::clamp(power, -540.0, 540.0);
//         }

//         pros::lcd::print(
//             5, "power: %.lf",
//             power);

//         // Move the motors
//         if (targetDistance_Y >= 0)
//         {
//             pros::lcd::print(
//                 0, "targetDistance_Y: %.lf",
//                 targetDistance_Y);
//             // luA.move_velocity(-power - angle_pid - offset_correction);
//             // luB.move_velocity(-power - angle_pid - offset_correction);
//             // llA.move_velocity(-power + angle_pid + offset_correction);
//             // llB.move_velocity(-power + angle_pid + offset_correction);
//             // ruA.move_velocity(-power - angle_pid - offset_correction);
//             // ruB.move_velocity(-power - angle_pid - offset_correction);
//             // rlA.move_velocity(-power + angle_pid + offset_correction);
//             // rlB.move_velocity(-power + angle_pid + offset_correction);

//             luA.move_velocity(power + angle_pid - offset_correction);
//             luB.move_velocity(power + angle_pid - offset_correction);
//             llA.move_velocity(power - angle_pid + offset_correction);
//             llB.move_velocity(power - angle_pid + offset_correction);
//             ruA.move_velocity(power + angle_pid - offset_correction);
//             ruB.move_velocity(power + angle_pid - offset_correction);
//             rlA.move_velocity(power - angle_pid + offset_correction);
//             rlB.move_velocity(power - angle_pid + offset_correction);
//             if (errordistance < 0)
//             {
//                 brake();
//                 break;
//             }
//         }
//         else
//         {
//             pros::lcd::print(
//                 0, "targetDistance_Y: %.lf",
//                 targetDistance_Y);
//             // luA.move_velocity(-power - angle_pid - offset_correction);
//             // luB.move_velocity(-power - angle_pid - offset_correction);
//             // llA.move_velocity(-power + angle_pid + offset_correction);
//             // llB.move_velocity(-power + angle_pid + offset_correction);
//             // ruA.move_velocity(-power - angle_pid - offset_correction);
//             // ruB.move_velocity(-power - angle_pid - offset_correction);
//             // rlA.move_velocity(-power + angle_pid + offset_correction);
//             // rlB.move_velocity(-power + angle_pid + offset_correction);

//             luA.move_velocity(power + angle_pid - offset_correction);
//             luB.move_velocity(power + angle_pid - offset_correction);
//             llA.move_velocity(power - angle_pid + offset_correction);
//             llB.move_velocity(power - angle_pid + offset_correction);
//             ruA.move_velocity(power + angle_pid - offset_correction);
//             ruB.move_velocity(power + angle_pid - offset_correction);
//             rlA.move_velocity(power - angle_pid + offset_correction);
//             rlB.move_velocity(power - angle_pid + offset_correction);
//             if (errordistance > 0)
//             {
//                 brake();
//                 break;
//             }
//         }

//         if (timeout >= 100000)
//             break;

//         if (fabs(power) < 2)
//         {
//             break;
//         }

//         pros::lcd::print(1, "Error: %.lf", errordistance);
//         pros::lcd::print(6, "angle_pid: %.lf", angle_pid);
//         pros::lcd::print(7, "offset_correction: %.lf", offset_correction);
//         pros::delay(2);
//         // pros::lcd::print(1, "ErrorR: %.lf", errorRight);

//         // // Update previous errors for the next iteration
//         // prevErrorLeft = errorLeft;
//         // prevErrorRight = errorRight;

//         // Delay to reduce CPU load
//         // timeout += 2;
//     }
//     // brake();
// }

void base_PID_front_back_imu_x(double base_kp, double base_ki, double base_kd,
                               double targetangle, double targetDistance_Y,
                               double decelerationThreshold, double angle_offset_p, double angle_offset_kd)
{
    // Movement variables
    double powerL = 0;
    double powerR = 0;
    double errorLeft = 0;
    double errorRight = 0;
    double prevErrorLeft = 0;
    double prevErrorRight = 0;
    double totalErrorLeft = 0;
    double totalErrorRight = 0;

    double offset_error = 0;
    double prev_offset_error = 0;
    double offset_correction = 0;

    double l_angleMaintain =
        (targetangle - 90.0) *
        TO_RADIANS; // note that the function getNormalizedSensorAngle already
                    // implements wrapAngle to bound the angle between -180 and
                    // 180 degrees
    double r_angleMaintain =
        (targetangle - 90.0) *
        TO_RADIANS; // note that the function getNormalizedSensorAngle already
                    // implements wrapAngle to bound the angle between -180 and
                    // 180 degrees

    double angleMaintain =
        (targetangle - 90.0) *
        TO_RADIANS; // note that the function getNormalizedSensorAngle already
                    // implements wrapAngle to bound the angle between -180 and
                    // 180 degrees
    double angle_pid = 0.0;
    double left_angle, right_angle;
    double l_error = 0.0;
    double r_error = 0.0;
    // power output for angle component of pid
    double l_angle_pid = 0.0;
    double r_angle_pid = 0.0;
    // 1. Perform turning first

    bool l_move = true;
    bool r_move = true;
    int timeout = 0;

    PID left_angle_PID(angle_kP, angle_kI, angle_kD);
    PID right_angle_PID(angle_kP, angle_kI, angle_kD);

    PID angle_PID(angle_offset_p, angle_kI, angle_offset_kd);

    double aim_target = targetDistance_Y - global_distY;
    original_x = global_distX;

    vector3D current_left_vector; // direction unit vector for wheels
    vector3D current_right_vector;
    double heading_error;
    double prev_errorLeft;
    double prev_errorRight;
    double initialHeading;

    while (l_move || r_move)
    {

        left_angle = wrapAngle(getNormalizedSensorAngle(left_rotation_sensor) - 90.0) * TO_RADIANS;   // takes robot right as 0
        right_angle = wrapAngle(getNormalizedSensorAngle(right_rotation_sensor) - 90.0) * TO_RADIANS; // Y axis positive is front
        imu_angle = (wrapAngle(imu.get_heading()) - 90.0)   * TO_RADIANS;
        current_left_vector = vector3D(cos(left_angle), sin(left_angle), 0.0);
        current_right_vector = vector3D(cos(right_angle), sin(right_angle), 0.0);

        vector3D imu_target_angle_vector =
            vector3D(cos(angleMaintain), sin(angleMaintain), 0);
        vector3D l_target_angle = vector3D(cos(l_angleMaintain), sin(l_angleMaintain), 0);
        vector3D r_target_angle = vector3D(cos(r_angleMaintain), sin(r_angleMaintain), 0);

        vector3D current_angle = vector3D(cos(imu_angle), sin(imu_angle), 0);
        vector3D l_current_angle = vector3D(cos(left_angle), sin(left_angle), 0);
        vector3D r_current_angle = vector3D(cos(right_angle), sin(right_angle), 0);

        l_error = angle(l_current_angle, l_target_angle);
        r_error = angle(r_current_angle, r_target_angle);
        heading_error = angle(current_angle, imu_target_angle_vector);

        // calculate the PID output
        l_angle_pid = left_angle_PID.step(l_error);
        r_angle_pid = right_angle_PID.step(r_error);
        angle_pid = angle_PID.step(heading_error);
        double targetDistance = aim_target - global_distY;
        pros::lcd::print(
            6, "current_distance: %.lf",
            targetDistance);

        errorLeft = targetDistance;
        errorRight = targetDistance;
        float errorLeft_derivative = errorLeft - prevErrorLeft;
        float errorRight_derivative = errorLeft - prevErrorRight;
        prevErrorLeft = errorLeft;
        prevErrorRight = errorRight;
        totalErrorLeft += errorLeft;
        totalErrorRight += errorRight;

        // offset_error = global_distX - original_x;
        // float offset_derivative = offset_error - prev_offset_error;
        // prev_offset_error = offset_error;
        // offset_correction = (offset_kp * offset_error) + (offset_kd * offset_derivative);

        // PID for left motors
        if (fabs(errorLeft) <= base_error)
        {
            powerL = 0.0;
            l_move = false;
        }
        else
        {
            // Gradually reduce power as you approach the target
            if (fabs(errorLeft) < decelerationThreshold)
            {
                powerL *= 0.5; // Reduce power to half when close to target
            }
            else
            {
                powerL = base_kp * errorLeft + base_ki * totalErrorLeft +
                         base_kd * errorLeft_derivative;
            }
            powerL = std::clamp(powerL, -540.0, 540.0);
        }

        // PID for right motors
        if (fabs(errorRight) <= base_error)
        {
            powerR = 0.0;
            r_move = false;
        }
        else
        {
            // Gradually reduce power as you approach the target
            if (fabs(errorRight) < decelerationThreshold)
            {
                powerR *= 0.5; // Reduce power to half when close to target
            }
            else
            {
                powerR = base_kp * errorRight + base_ki * totalErrorRight +
                         base_kd * errorRight_derivative;
            }
            powerR = std::clamp(powerR, -540.0, 540.0);
        }
        pros::lcd::print(
            5, "powerL: %.lf",
            powerL);
        // pros::lcd::print(
        //     6, "powerR: %.lf",
        //     powerR);
        // Move the motors
        if (targetDistance_Y >= 0)
        {
            pros::lcd::print(
                0, "targetDistance_Y: %.lf",
                targetDistance_Y);
            luA.move_velocity(powerL - l_angle_pid + angle_pid);
            luB.move_velocity(powerL - l_angle_pid + angle_pid);
            llA.move_velocity(powerL + l_angle_pid + angle_pid);
            llB.move_velocity(powerL + l_angle_pid + angle_pid);
            ruA.move_velocity(powerR - r_angle_pid - angle_pid);
            ruB.move_velocity(powerR - r_angle_pid - angle_pid);
            rlA.move_velocity(powerR + r_angle_pid - angle_pid);
            rlB.move_velocity(powerR + r_angle_pid - angle_pid);

            // luA.move_velocity(powerL - l_angle_pid - angle_pid);
            // luB.move_velocity(powerL - l_angle_pid - angle_pid);
            // llA.move_velocity(powerL + l_angle_pid - angle_pid);
            // llB.move_velocity(powerL + l_angle_pid - angle_pid);
            // ruA.move_velocity(powerR - r_angle_pid + angle_pid);
            // ruB.move_velocity(powerR - r_angle_pid + angle_pid);
            // rlA.move_velocity(powerR + r_angle_pid + angle_pid);
            // rlB.move_velocity(powerR + r_angle_pid + angle_pid);
            if (errorRight < 0 && errorLeft < 0)
            {
                brake();
                break;
            }
        }
        else
        {
            pros::lcd::print(
                0, "targetDistance_Y: %.lf",
                targetDistance_Y);
            luA.move_velocity(powerL - l_angle_pid + angle_pid);
            luB.move_velocity(powerL - l_angle_pid + angle_pid);
            llA.move_velocity(powerL + l_angle_pid + angle_pid);
            llB.move_velocity(powerL + l_angle_pid + angle_pid);
            ruA.move_velocity(powerR - r_angle_pid - angle_pid);
            ruB.move_velocity(powerR - r_angle_pid - angle_pid);
            rlA.move_velocity(powerR + r_angle_pid - angle_pid);
            rlB.move_velocity(powerR + r_angle_pid - angle_pid);

            // luA.move_velocity(powerL - l_angle_pid - angle_pid);
            // luB.move_velocity(powerL - l_angle_pid - angle_pid);
            // llA.move_velocity(powerL + l_angle_pid - angle_pid);
            // llB.move_velocity(powerL + l_angle_pid - angle_pid);
            // ruA.move_velocity(powerR - r_angle_pid + angle_pid);
            // ruB.move_velocity(powerR - r_angle_pid + angle_pid);
            // rlA.move_velocity(powerR + r_angle_pid + angle_pid);
            // rlB.move_velocity(powerR + r_angle_pid + angle_pid);
            if (errorRight > 0 && errorLeft > 0)
            {
                brake();
                break;
            }
        }

        if (timeout >= 100000)
            break;

        if (fabs(powerR) < 2 || fabs(powerL) < 2)
        {
            break;
        }

        // pros::lcd::print(0, "ErrorL: %.lf", errorLeft);
        pros::lcd::print(1, "ErrorR: %.lf", errorRight);

        // Update previous errors for the next iteration

        pros::delay(2); // Delay to reduce CPU load
        timeout += 2;
    }
    brake();
}

void base_PID_left_right(double base_kp, double base_ki, double base_kd,
                         double targetangle, double targetDistance_X = 0,
                         double decelerationThreshold = 0)
{
    // Movement variables
    double powerL = 0;
    double powerR = 0;
    double encdleft = 0;
    double encdright = 0;
    double errorLeft = 0;
    double errorRight = 0;
    double prevErrorLeft = 0;
    double prevErrorRight = 0;
    double totalErrorLeft = 0;
    double totalErrorRight = 0;

    double l_angleMaintain =
        (targetangle - 90.0) *
        TO_RADIANS; // note that the function getNormalizedSensorAngle already
                    // implements wrapAngle to bound the angle between -180 and
                    // 180 degrees
    double r_angleMaintain =
        (targetangle - 90.0) *
        TO_RADIANS; // note that the function getNormalizedSensorAngle already
                    // implements wrapAngle to bound the angle between -180 and
                    // 180 degrees

    double left_angle, right_angle;
    double l_error = 0.0;
    double r_error = 0.0;
    // power output for angle component of pid
    double l_angle_pid = 0.0;
    double r_angle_pid = 0.0;
    // 1. Perform turning first

    bool l_move = true;
    bool r_move = true;
    int timeout = 0;

    PID left_angle_PID(angle_kP_mehmeh, angle_kI, angle_kD);
    PID right_angle_PID(angle_kP_mehmeh, angle_kI, angle_kD);

    double aim_target = targetDistance_X - global_distX;
    double original_y = global_distY;

    while (l_move || r_move)
    {

        left_angle = (getNormalizedSensorAngle(left_rotation_sensor) - 90.0) *
                     TO_RADIANS; // note that the function getNormalizedSensorAngle
                                 // already implements wrapAngle to bound the angle
                                 // between -180 and 180 degrees
        right_angle =
            (getNormalizedSensorAngle(right_rotation_sensor) - 90.0) * TO_RADIANS;

        vector3D l_target_angle =
            vector3D(cos(l_angleMaintain), sin(l_angleMaintain), 0);
        vector3D r_target_angle =
            vector3D(cos(r_angleMaintain), sin(r_angleMaintain), 0);
        vector3D l_current_angle = vector3D(cos(left_angle), sin(left_angle), 0);
        vector3D r_current_angle = vector3D(cos(right_angle), sin(right_angle), 0);

        l_error = angle(l_current_angle, l_target_angle);
        r_error = angle(r_current_angle, r_target_angle);

        // calculate the PID output
        l_angle_pid = left_angle_PID.step(l_error);
        r_angle_pid = right_angle_PID.step(r_error);
        // Calculate distance error

        double targetDistance = aim_target - global_distX;
        pros::lcd::print(
            4, "l_angle_pid: %.lf",
            l_angle_pid);

        errorLeft = targetDistance;
        errorRight = targetDistance;
        totalErrorLeft += errorLeft;
        totalErrorRight += errorRight;

        // PID for left motors
        if (fabs(errorLeft) <= base_error)
        {
            powerL = 0.0;
            l_move = false;
        }
        else
        {
            // Gradually reduce power as you approach the target
            if (fabs(errorLeft) < decelerationThreshold)
            {
                powerL *= 0.5; // Reduce power to half when close to target
            }
            else
            {
                powerL = base_kp * errorLeft + base_ki * totalErrorLeft +
                         base_kd * (errorLeft - prevErrorLeft);
            }
            powerL = std::clamp(powerL, -200.0, 300.0);
        }

        // PID for right motors
        if (fabs(errorRight) <= base_error)
        {
            powerR = 0.0;
            r_move = false;
        }
        else
        {
            // Gradually reduce power as you approach the target
            if (fabs(errorRight) < decelerationThreshold)
            {
                powerR *= 0.5; // Reduce power to half when close to target
            }
            else
            {
                powerR = base_kp * errorRight + base_ki * totalErrorRight +
                         base_kd * (errorRight - prevErrorRight);
            }
            powerR = std::clamp(powerR, -200.0, 300.0);
        }
        pros::lcd::print(
            5, "powerL: %.lf",
            powerL);
        pros::lcd::print(
            6, "powerR: %.lf",
            powerR);

        // Move the motors

        if (fabs(powerR) < 2 || fabs(powerL) < 2)
        {
            break;
        }

        if (targetDistance_X >= 0)
        {

            // luA.move_velocity(powerL - l_angle_pid);
            // luB.move_velocity(powerL - l_angle_pid);
            // llA.move_velocity(powerL + l_angle_pid);
            // llB.move_velocity(powerL + l_angle_pid);
            // ruA.move_velocity(powerR - r_angle_pid);
            // ruB.move_velocity(powerR - r_angle_pid);
            // rlA.move_velocity(powerR + r_angle_pid);
            // rlB.move_velocity(powerR + r_angle_pid);

            // luA.move_velocity(powerL + l_angle_pid);
            // luB.move_velocity(powerL + l_angle_pid);
            // llA.move_velocity(powerL - l_angle_pid);
            // llB.move_velocity(powerL - l_angle_pid);
            // ruA.move_velocity(powerR + r_angle_pid);
            // ruB.move_velocity(powerR + r_angle_pid);
            // rlA.move_velocity(powerR - r_angle_pid);
            // rlB.move_velocity(powerR - r_angle_pid);

            // luA.move_velocity(-powerL - l_angle_pid);
            // luB.move_velocity(-powerL - l_angle_pid);
            // llA.move_velocity(-powerL + l_angle_pid);
            // llB.move_velocity(-powerL + l_angle_pid);
            // ruA.move_velocity(-powerR - r_angle_pid);
            // ruB.move_velocity(-powerR - r_angle_pid);
            // rlA.move_velocity(-powerR + r_angle_pid);
            // rlB.move_velocity(-powerR + r_angle_pid);

            luA.move_velocity(-powerL + l_angle_pid);
            luB.move_velocity(-powerL + l_angle_pid);
            llA.move_velocity(-powerL - l_angle_pid);
            llB.move_velocity(-powerL - l_angle_pid);
            ruA.move_velocity(-powerR + r_angle_pid);
            ruB.move_velocity(-powerR + r_angle_pid);
            rlA.move_velocity(-powerR - r_angle_pid);
            rlB.move_velocity(-powerR - r_angle_pid);

            if (errorRight < 0 && errorLeft < 0)

            // if (errorRight > 0 && errorLeft > 0)
            {
                brake();
                break;
            }
        }
        else
        {
            // luA.move_velocity(powerL - l_angle_pid);
            // luB.move_velocity(powerL - l_angle_pid);
            // llA.move_velocity(powerL + l_angle_pid);
            // llB.move_velocity(powerL + l_angle_pid);
            // ruA.move_velocity(powerR - r_angle_pid);
            // ruB.move_velocity(powerR - r_angle_pid);
            // rlA.move_velocity(powerR + r_angle_pid);
            // rlB.move_velocity(powerR + r_angle_pid);

            // luA.move_velocity(powerL + l_angle_pid);
            // luB.move_velocity(powerL + l_angle_pid);
            // llA.move_velocity(powerL - l_angle_pid);
            // llB.move_velocity(powerL - l_angle_pid);
            // ruA.move_velocity(powerR + r_angle_pid);
            // ruB.move_velocity(powerR + r_angle_pid);
            // rlA.move_velocity(powerR - r_angle_pid);
            // rlB.move_velocity(powerR - r_angle_pid);

            luA.move_velocity(-powerL + l_angle_pid);
            luB.move_velocity(-powerL + l_angle_pid);
            llA.move_velocity(-powerL - l_angle_pid);
            llB.move_velocity(-powerL - l_angle_pid);
            ruA.move_velocity(-powerR + r_angle_pid);
            ruB.move_velocity(-powerR + r_angle_pid);
            rlA.move_velocity(-powerR - r_angle_pid);
            rlB.move_velocity(-powerR - r_angle_pid);

            // luA.move_velocity(-powerL - l_angle_pid);
            // luB.move_velocity(-powerL - l_angle_pid);
            // llA.move_velocity(-powerL + l_angle_pid);
            // llB.move_velocity(-powerL + l_angle_pid);
            // ruA.move_velocity(-powerR - r_angle_pid);
            // ruB.move_velocity(-powerR - r_angle_pid);
            // rlA.move_velocity(-powerR + r_angle_pid);
            // rlB.move_velocity(-powerR + r_angle_pid);

            // if (errorRight < 0 && errorLeft < 0)
            if (errorRight > 0 && errorLeft > 0)
            {
                brake();
                break;
            }
        }

        if (timeout >= 100000)
            break;

        pros::lcd::print(0, "ErrorL: %.lf", errorLeft);
        pros::lcd::print(1, "ErrorR: %.lf", errorRight);

        // Update previous errors for the next iteration
        prevErrorLeft = errorLeft;
        prevErrorRight = errorRight;

        pros::delay(2); // Delay to reduce CPU load
        timeout += 2;
    }
    brake();
}

void base_PID_left_right_flipped(double base_kp, double base_ki, double base_kd,
                                 double targetangle, double targetDistance_X = 0,
                                 double decelerationThreshold = 0)
{
    // Movement variables
    double powerL = 0;
    double powerR = 0;
    double encdleft = 0;
    double encdright = 0;
    double errorLeft = 0;
    double errorRight = 0;
    double prevErrorLeft = 0;
    double prevErrorRight = 0;
    double totalErrorLeft = 0;
    double totalErrorRight = 0;

    double l_angleMaintain =
        (targetangle - 90.0) *
        TO_RADIANS; // note that the function getNormalizedSensorAngle already
                    // implements wrapAngle to bound the angle between -180 and
                    // 180 degrees
    double r_angleMaintain =
        (targetangle - 90.0) *
        TO_RADIANS; // note that the function getNormalizedSensorAngle already
                    // implements wrapAngle to bound the angle between -180 and
                    // 180 degrees

    double left_angle, right_angle;
    double l_error = 0.0;
    double r_error = 0.0;
    // power output for angle component of pid
    double l_angle_pid = 0.0;
    double r_angle_pid = 0.0;
    // 1. Perform turning first

    bool l_move = true;
    bool r_move = true;
    int timeout = 0;

    PID left_angle_PID(angle_kP_mehmeh, angle_kI, angle_kD);
    PID right_angle_PID(angle_kP_mehmeh, angle_kI, angle_kD);

    double aim_target = targetDistance_X - global_distX;
    double original_y = global_distY;

    while (l_move || r_move)
    {

        left_angle = (getNormalizedSensorAngle(left_rotation_sensor) - 90.0) *
                     TO_RADIANS; // note that the function getNormalizedSensorAngle
                                 // already implements wrapAngle to bound the angle
                                 // between -180 and 180 degrees
        right_angle =
            (getNormalizedSensorAngle(right_rotation_sensor) - 90.0) * TO_RADIANS;

        vector3D l_target_angle =
            vector3D(cos(l_angleMaintain), sin(l_angleMaintain), 0);
        vector3D r_target_angle =
            vector3D(cos(r_angleMaintain), sin(r_angleMaintain), 0);
        vector3D l_current_angle = vector3D(cos(left_angle), sin(left_angle), 0);
        vector3D r_current_angle = vector3D(cos(right_angle), sin(right_angle), 0);

        l_error = angle(l_current_angle, l_target_angle);
        r_error = angle(r_current_angle, r_target_angle);

        // calculate the PID output
        l_angle_pid = left_angle_PID.step(l_error);
        r_angle_pid = right_angle_PID.step(r_error);
        // Calculate distance error

        double targetDistance = aim_target - global_distX;
        pros::lcd::print(
            4, "l_angle_pid: %.lf",
            l_angle_pid);

        errorLeft = targetDistance;
        errorRight = targetDistance;
        totalErrorLeft += errorLeft;
        totalErrorRight += errorRight;

        // PID for left motors
        if (fabs(errorLeft) <= base_error)
        {
            powerL = 0.0;
            l_move = false;
        }
        else
        {
            // Gradually reduce power as you approach the target
            if (fabs(errorLeft) < decelerationThreshold)
            {
                powerL *= 0.5; // Reduce power to half when close to target
            }
            else
            {
                powerL = base_kp * errorLeft + base_ki * totalErrorLeft +
                         base_kd * (errorLeft - prevErrorLeft);
            }
            powerL = std::clamp(powerL, -200.0, 300.0);
        }

        // PID for right motors
        if (fabs(errorRight) <= base_error)
        {
            powerR = 0.0;
            r_move = false;
        }
        else
        {
            // Gradually reduce power as you approach the target
            if (fabs(errorRight) < decelerationThreshold)
            {
                powerR *= 0.5; // Reduce power to half when close to target
            }
            else
            {
                powerR = base_kp * errorRight + base_ki * totalErrorRight +
                         base_kd * (errorRight - prevErrorRight);
            }
            powerR = std::clamp(powerR, -200.0, 300.0);
        }
        pros::lcd::print(
            5, "powerL: %.lf",
            powerL);
        pros::lcd::print(
            6, "powerR: %.lf",
            powerR);
        // Move the motors

        if (fabs(powerR) < 2 || fabs(powerL) < 2)
        {
            break;
        }

        if (targetDistance_X >= 0)
        {

            // luA.move_velocity(powerL - l_angle_pid);
            // luB.move_velocity(powerL - l_angle_pid);
            // llA.move_velocity(powerL + l_angle_pid);
            // llB.move_velocity(powerL + l_angle_pid);
            // ruA.move_velocity(powerR - r_angle_pid);
            // ruB.move_velocity(powerR - r_angle_pid);
            // rlA.move_velocity(powerR + r_angle_pid);
            // rlB.move_velocity(powerR + r_angle_pid);

            // luA.move_velocity(powerL + l_angle_pid);
            // luB.move_velocity(powerL + l_angle_pid);
            // llA.move_velocity(powerL - l_angle_pid);
            // llB.move_velocity(powerL - l_angle_pid);
            // ruA.move_velocity(powerR + r_angle_pid);
            // ruB.move_velocity(powerR + r_angle_pid);
            // rlA.move_velocity(powerR - r_angle_pid);
            // rlB.move_velocity(powerR - r_angle_pid);

            // luA.move_velocity(-powerL - l_angle_pid);
            // luB.move_velocity(-powerL - l_angle_pid);
            // llA.move_velocity(-powerL + l_angle_pid);
            // llB.move_velocity(-powerL + l_angle_pid);
            // ruA.move_velocity(-powerR - r_angle_pid);
            // ruB.move_velocity(-powerR - r_angle_pid);
            // rlA.move_velocity(-powerR + r_angle_pid);
            // rlB.move_velocity(-powerR + r_angle_pid);

            luA.move_velocity(powerL + l_angle_pid);
            luB.move_velocity(powerL + l_angle_pid);
            llA.move_velocity(powerL - l_angle_pid);
            llB.move_velocity(powerL - l_angle_pid);
            ruA.move_velocity(powerR + r_angle_pid);
            ruB.move_velocity(powerR + r_angle_pid);
            rlA.move_velocity(powerR - r_angle_pid);
            rlB.move_velocity(powerR - r_angle_pid);

            if (errorRight < 0 && errorLeft < 0)

            // if (errorRight > 0 && errorLeft > 0)
            {
                brake();
                break;
            }
        }
        else
        {
            // luA.move_velocity(powerL - l_angle_pid);
            // luB.move_velocity(powerL - l_angle_pid);
            // llA.move_velocity(powerL + l_angle_pid);
            // llB.move_velocity(powerL + l_angle_pid);
            // ruA.move_velocity(powerR - r_angle_pid);
            // ruB.move_velocity(powerR - r_angle_pid);
            // rlA.move_velocity(powerR + r_angle_pid);
            // rlB.move_velocity(powerR + r_angle_pid);

            // luA.move_velocity(powerL + l_angle_pid);
            // luB.move_velocity(powerL + l_angle_pid);
            // llA.move_velocity(powerL - l_angle_pid);
            // llB.move_velocity(powerL - l_angle_pid);
            // ruA.move_velocity(powerR + r_angle_pid);
            // ruB.move_velocity(powerR + r_angle_pid);
            // rlA.move_velocity(powerR - r_angle_pid);
            // rlB.move_velocity(powerR - r_angle_pid);

            luA.move_velocity(powerL + l_angle_pid);
            luB.move_velocity(powerL + l_angle_pid);
            llA.move_velocity(powerL - l_angle_pid);
            llB.move_velocity(powerL - l_angle_pid);
            ruA.move_velocity(powerR + r_angle_pid);
            ruB.move_velocity(powerR + r_angle_pid);
            rlA.move_velocity(powerR - r_angle_pid);
            rlB.move_velocity(powerR - r_angle_pid);

            // luA.move_velocity(-powerL - l_angle_pid);
            // luB.move_velocity(-powerL - l_angle_pid);
            // llA.move_velocity(-powerL + l_angle_pid);
            // llB.move_velocity(-powerL + l_angle_pid);
            // ruA.move_velocity(-powerR - r_angle_pid);
            // ruB.move_velocity(-powerR - r_angle_pid);
            // rlA.move_velocity(-powerR + r_angle_pid);
            // rlB.move_velocity(-powerR + r_angle_pid);

            // if (errorRight < 0 && errorLeft < 0)
            if (errorRight > 0 && errorLeft > 0)
            {
                brake();
                break;
            }
        }

        if (timeout >= 100000)
            break;

        pros::lcd::print(0, "ErrorL: %.lf", errorLeft);
        pros::lcd::print(1, "ErrorR: %.lf", errorRight);

        // Update previous errors for the next iteration
        prevErrorLeft = errorLeft;
        prevErrorRight = errorRight;

        pros::delay(2); // Delay to reduce CPU load
        timeout += 2;
    }
    brake();
}

void initialize()
{
    pros::lcd::initialize();
    imu.reset();

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

    // while(!left_rotation_sensor.reset());
    // while(!right_rotation_sensor.reset());

    left_rotation_sensor.set_data_rate(5);
    right_rotation_sensor.set_data_rate(5);

    left_rotation_sensor.set_position(0);
    right_rotation_sensor.set_position(0);

    conveyor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    roller.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    // pros::Task move_base(moveBase);
    pros::Task serial_read(serialRead);
    // pros::Task mehmehisreal(printmehmeh);

    master.clear();
}

void autonomous()
{

    // set_wheel_angle(0, 0.5, 0.00, 0.1);

    // base_PID_front_back(1.5, 0, 0,  0, -900,295, 0.5, 0.0);
    // base_PID_front_back(1.5, 0, 0, 0, 860, 250, 0.5, 0.0);
    // base_PID_front_back_flipped(1.5, 0, 1.5, 0.1, 0, 800, 300, 0.5, 0.0);
    // set_wheel_angle(-90, 1.5, 0.00, 0.4);

    // // base_PID_left_right(1, 0, 0, 90, -300, 220);

    // base_PID_left_right_flipped(1, 0, 0, 90, 300, 70);
    // set_wheel_angle(1, 1.5, 0.00, 0.4);
    // base_PID_front_back_flipped(1.5, 0, 1.5, 0.1, 0, 725, 0, 0.5, 0.0);

    // base_PID(0.025, 0, 0, false, 0, 700,100);
    // set_wheel_angle(0, 0.5, 0.00, 0.1);
    // base_PID(700,5.0,0,0.2 );
}

void autonomousb()
{
    // set_wheel_angle_new(0, 0.1, 0, 0);

    // base_PID_front_back_flipped(1.5, 0,1.5, 0, 0, 700, 400, 18, 5000.0);
    base_PID_front_back(0.05, 0, 0, 0, 1200, 0, 0.5, 0.0);

    // mainnn
    // base_PID_front_back_flipped(1.5, 0, 1.5, 0, 0, 750, 370, 0.5, 0.0);

    // base_PID_front_back_decelerate(1.5, 0, 0, 0, 750, 370, 18, 5000.0);
    // set_wheel_angle_new(90, 2.2, 0, 0.1);

    //     // set_wheel_angle(-90, 1.5, 0.00, 0.4);
    //       mobilegoal_bot.set_value(0);
    //         pros::Task::delay(100);
    //         solenoid.set_value(1);
    // set_wheel_angle(-90, 0.05, 0.00, 0.1);
    // base_PID_left_right(1, 0, 0, 90, 300, 220);
    // set_wheel_angle(-90, 0.05, 0.00, 0.1);
}

void autonomousy()
{
    // set_wheel_angle_new(0, 0.2, 0, 0.2);
    // base_PID_front_back(0.05, 0, 0, 0, 10, 0, 0.5, 0.0);
    base_PID_front_back_imu_x(2.5, 0, 0, 0, 350, 0, 0.3, 0.1);
    // set_wheel_angle_new(0, 0.2, 0, 0.2);
    base_PID_front_back_imu_x(1.5, 0, 0, 0, 850, 0, 0.4, 0.2);
    // base_PID_front_back_imu_x(1.5, 0, 0, 0, 1000, 0, 0.4, 0.2);


    // set_wheel_angle_new(90, 2.2, 0, 0.1);
    // turn_angle(-45, 1.5, 0.1);

    // set_wheel_angle_new(90,2.2,0,0.1);

    // Control update rate

    //   set_wheel_angle(-90, 1.5, 0.00, 0.4);

    //   base_PID_left_right(1, 0, 0, 90, -300, 230);
    //   set_wheel_angle(-90, 0.05, 0.00, 0.1);
    //   // base_PID(1000, 0, 0, false, 0, 500,400);

    //   // base_PID(0.025, 0, 0, false, 0, -700,100);
    //   set_wheel_angle(0, 0.5, 0.00, 0.1);
    //   base_PID(700,5.0,0,0.2 );
}

void opcontrol()
{
    while (true)
    {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X))
        {
            autonomous();
        }
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B))
        {
            autonomousb();
        }
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y))

        {
            pros::lcd::print(0, "yyyyyyyyyyyyyy");
            autonomousy();
        }
        if (master.get_digital_new_press(DIGITAL_A))
            mobile_goal_actuated = !mobile_goal_actuated;
        if (mobile_goal_actuated)
        {

            mobilegoal_bot.set_value(0);
            pros::Task::delay(100);
            solenoid.set_value(1);
        }

        else
        {

            mobilegoal_bot.set_value(1);
            pros::Task::delay(100);
            solenoid.set_value(0);
        }
        pros::delay(5);
    }
}
