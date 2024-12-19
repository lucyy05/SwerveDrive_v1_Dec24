// #include "main.h"
// #include "pros/misc.h"

// void disabled() {}
// void competition_initialize() {}

// // Function to determine sign of a integer variable, returns bool
// template <typename T>
// int sgn(T val) { return (T(0) < val) - (val < T(0)); }

// void serialRead(void *params)
// {
//     vexGenericSerialEnable(SERIALPORT - 1, 0);
//     vexGenericSerialBaudrate(SERIALPORT - 1, 115200);
//     pros::delay(10);
//     pros::screen::set_pen(COLOR_BLUE);
//     double distX, distY = 0;
//     while (true)
//     {
//         uint8_t buffer[256];
//         int bufLength = 256;
//         int32_t nRead = vexGenericSerialReceive(SERIALPORT - 1, buffer, bufLength);
//         if (nRead >= 0)
//         {
//             std::stringstream dataStream("");
//             bool recordOpticalX, recordOpticalY = false;
//             for (int i = 0; i < nRead; i++)
//             {
//                 char thisDigit = (char)buffer[i];
//                 if (thisDigit == 'D' || thisDigit == 'I' || thisDigit == 'A' ||
//                     thisDigit == 'X' || thisDigit == 'C' || thisDigit == 'Y')
//                 {
//                     recordOpticalX = false;
//                     recordOpticalY = false;
//                 }
//                 if (thisDigit == 'C')
//                 {
//                     recordOpticalX = false;
//                     dataStream >> distX;
//                     // pros::lcd::print(1, "Optical Flow:");
//                     global_distX = distX * 10;
//                     pros::lcd::print(2, "distX: %.2lf", distX * 10);
//                     dataStream.str(std::string());
//                 }
//                 if (thisDigit == 'D')
//                 {
//                     recordOpticalY = false;
//                     dataStream >> distY;
//                     global_distY = distY * 10;
//                     pros::lcd::print(3, "distY: %.2lf", global_distY);
//                     dataStream.str(std::string());
//                 }
//                 if (recordOpticalX)
//                     dataStream << (char)buffer[i];
//                 if (recordOpticalY)
//                     dataStream << (char)buffer[i];
//                 if (thisDigit == 'X')
//                     recordOpticalX = true;
//                 if (thisDigit == 'Y')
//                     recordOpticalY = true;
//             }
//         }
//         pros::Task::delay(2);
//     }
// }

// void brake()
// { // brakes all base motors
//     luA.brake();
//     ruA.brake();
//     luB.brake();
//     ruB.brake();
//     llA.brake();
//     rlA.brake();
//     llB.brake();
//     rlB.brake();
//     pros::delay(1);
// }

// void turn_angle(double targetTurning, double turn_Kp, double turn_Kd)
// {
//     while (!imu.tare_rotation())
//         ;
//     double initialHeading = imu.get_rotation(); // Store the initial heading
//     double currentHeading = initialHeading;
//     double turnError = 0.0;

//     // 1. Perform turning first

//     double prevError = 0.0;
//     while (true)
//     {
//         currentHeading = fabs(imu.get_rotation());
//         turnError = fabs(fabs(targetTurning) - (currentHeading));
//         pros::lcd::print(0, "Error: %.lf", turnError);

//         // Check if we are within the error threshold
//         if (fabs(turnError) <= 1.0)
//         {
//             // Stop turning if we are close enough
//             brake();
//             luA.move(0); // Adjust left side for turning
//             luB.move(0);
//             llA.move(0);
//             llA.move(0);
//             ruA.move(0); // Adjust right side for turning
//             ruB.move(0);
//             rlA.move(0);
//             rlB.move(0);
//             break; // Exit turning loop
//         }
//         double turnDerivative = prevError - turnError;

//         // Calculate turn power
//         double turnPower =
//             turnError * turn_Kp + turnDerivative * turn_Kd; // Tune this gain
//         prevError = turnError;

//         // Adjust motor powers for turning
//         if (targetTurning > 0)
//         {
//             luA.move(turnPower); // Adjust left side for turning
//             luB.move(turnPower);
//             llA.move(turnPower);
//             llA.move(turnPower);
//             ruA.move(-turnPower); // Adjust right side for turning
//             ruB.move(-turnPower);
//             rlA.move(-turnPower);
//             rlB.move(-turnPower);
//         }
//         else if (targetTurning < 0)
//         {
//             luA.move(-turnPower); // Adjust left side for turning
//             luB.move(-turnPower);
//             llA.move(-turnPower);
//             llA.move(-turnPower);
//             ruA.move(turnPower); // Adjust right side for turning
//             ruB.move(turnPower);
//             rlA.move(turnPower);
//             rlB.move(turnPower);
//         }

//         if (fabs(turnError) > fabs(targetTurning))
//         {
//             brake();
//             luA.move(0); // Adjust left side for turning
//             luB.move(0);
//             llA.move(0);
//             llA.move(0);
//             ruA.move(0); // Adjust right side for turning
//             ruB.move(0);
//             rlA.move(0);
//             rlB.move(0);
//             break;
//         }

//         pros::delay(5); // Delay to reduce CPU load
//     }
// }

// double wrapAngle(double angle)
// { // forces the angle to be within the -180 <
//     // angle < 180 range
//     if (angle > 180.0)
//         while (angle > 180.0)
//             angle -= 360.0;
//     else if (angle < -180.0)
//         while (angle < -180.0)
//             angle += 360.0;
//     return angle;
// }

// double getNormalizedSensorAngle(
//     pros::Rotation &sensor)
// { // Converts rotational sensor readings into
//     // degrees and bounds it between -180 to 180
//     double angle =
//         sensor.get_angle() / 100.0; // Convert from centidegrees to degrees
//     return wrapAngle(
//         angle); // forces the angle to be within the -180 < angle < 180 range
// }

// void set_wheel_angle(float target_angle, double kP_set_wheel = 2.0,
//                      double kI_set_wheel = 0.00, double kD_set_wheel = 0.2)
// {
//     float left_current_angle = getNormalizedSensorAngle(left_rotation_sensor);
//     float right_current_angle = getNormalizedSensorAngle(right_rotation_sensor);

//     float left_error = target_angle - left_current_angle;
//     float right_error = target_angle - right_current_angle;

//     float left_previous_error = 0;
//     float right_previous_error = 0;

//     float left_integral = 0;
//     float right_integral = 0;

//     // Normalize the errors to be within [-180, 180] degrees
//     while (left_error > 180)
//         left_error -= 360;
//     while (left_error < -180)
//         left_error += 360;
//     while (right_error > 180)
//         right_error -= 360;
//     while (right_error < -180)
//         right_error += 360;

//     int max_attempts = 50; // Set a maximum number of attempts to adjust the angle
//     int attempts = 0;

//     while ((fabs(left_error) > 2 || fabs(right_error) > 2) &&
//            attempts < max_attempts)
//     {
//         left_current_angle = getNormalizedSensorAngle(left_rotation_sensor);
//         right_current_angle = getNormalizedSensorAngle(right_rotation_sensor);

//         left_error = target_angle - left_current_angle;
//         right_error = target_angle - right_current_angle;

//         while (left_error > 180)
//             left_error -= 360;
//         while (left_error < -180)
//             left_error += 360;
//         while (right_error > 180)
//             right_error -= 360;
//         while (right_error < -180)
//             right_error += 360;

//         left_integral += left_error;
//         right_integral += right_error;

//         float left_derivative = left_error - left_previous_error;
//         float right_derivative = right_error - right_previous_error;

//         left_previous_error = left_error;
//         right_previous_error = right_error;

//         float left_motor_speed = kP_set_wheel * left_error + kD_set_wheel * left_derivative;
//         float right_motor_speed = kP_set_wheel * right_error + kD_set_wheel * right_derivative;
//         pros::lcd::print(0, "inside setwheel");

//         luA.move(left_motor_speed);
//         luB.move(left_motor_speed);
//         llA.move(-left_motor_speed);
//         llB.move(-left_motor_speed);

//         ruA.move(right_motor_speed);
//         ruB.move(right_motor_speed);
//         rlA.move(-right_motor_speed);
//         rlB.move(-right_motor_speed);

//         pros::delay(2); // Refresh rate within the set_wheel_angle function
//         attempts++;
//     }

//     // Stop the motors
//     luA.move(0);
//     luB.move(0);
//     llA.move(0);
//     llB.move(0);
//     ruA.move(0);
//     ruB.move(0);
//     rlA.move(0);
//     rlB.move(0);
// }

// void tareBaseMotorEncoderPositions() // tares all base motor encoder positions
// {
//     luA.tare_position();
//     ruA.tare_position();
//     luB.tare_position();
//     ruB.tare_position();
//     llA.tare_position();
//     rlA.tare_position();
//     llB.tare_position();
//     rlB.tare_position();
//     pros::delay(1);
// }

// void clampVoltage(double lu, double ll, double ru, double rl)
// {
//     // if any of lu, ll, ru or rl are too big, we need to scale them, and we must
//     // scale them all by the same amount so we dont throw off the proportions
//     if (fabs(lu) > MAX_VOLTAGE || fabs(ll) > MAX_VOLTAGE ||
//         fabs(ru) > MAX_VOLTAGE || fabs(rl) > MAX_VOLTAGE)
//     {
//         // figure out which of lu, ll, ru or rl has the largest magnitude
//         double max = fabs(lu);
//         if (max < fabs(ll))
//             max = fabs(ll);
//         if (max < fabs(ru))
//             max = fabs(ru);
//         if (max < fabs(rl))
//             max = fabs(rl);
//         double VoltageScalingFactor =
//             max / MAX_VOLTAGE; // this will definitely be positive, hence it wont
//                                // change the sign of lu, ll, ru or rl.
//         lu = lu / VoltageScalingFactor;
//         ll = ll / VoltageScalingFactor;
//         ru = ru / VoltageScalingFactor;
//         rl = rl / VoltageScalingFactor;
//     }
// }

// double angle(vector3D v1, vector3D v2)
// {
//     double dot = v1 * v2;
//     double det = v1.x * v2.y - v1.y * v2.x;
//     return -atan2(det, dot); // atan2 automatically considers the sign to deal
//                              // with the trigonometry quadrant
// }

// void update_turning(double turn_target_angle, double kP, double kD)
// {

//     is_turning = true;
//     turn_previous_error = 0;
//     double powerL = 0;
//     double powerR = 0;
//     double errorLeft = 0;
//     double errorRight = 0;
//     double prevErrorLeft = 0;
//     double prevErrorRight = 0;
//     double totalErrorLeft = 0;
//     double totalErrorRight = 0;

//     double offset_error = 0;
//     double prev_offset_error = 0;
//     double offset_correction = 0;

//     double l_angleMaintain =
//         (0 - 90.0) *
//         TO_RADIANS; // note that the function getNormalizedSensorAngle already
//                     // implements wrapAngle to bound the angle between -180 and
//                     // 180 degrees
//     double r_angleMaintain =
//         (0 - 90.0) *
//         TO_RADIANS; // note that the function getNormalizedSensorAngle already
//                     // implements wrapAngle to bound the angle between -180 and
//                     // 180 degrees

//     double left_angle, right_angle;
//     double l_error = 0.0;
//     double r_error = 0.0;
//     // power output for angle component of pid
//     double l_angle_pid = 0.0;
//     double r_angle_pid = 0.0;
//     // 1. Perform turning first

//     PID left_angle_PID(angle_kP, angle_kI, angle_kD);
//     PID right_angle_PID(angle_kP, angle_kI, angle_kD);
//     double current_angle = imu.get_heading();

//     double error = turn_target_angle - current_angle;

//     while (fabs(error) > 2)
//     {
//         left_angle = (getNormalizedSensorAngle(left_rotation_sensor) - 90.0) *
//                      TO_RADIANS; // note that the function getNormalizedSensorAngle
//                                  // already implements wrapAngle to bound the angle
//                                  // between -180 and 180 degrees
//         right_angle =
//             (getNormalizedSensorAngle(right_rotation_sensor) - 90.0) * TO_RADIANS;

//         vector3D l_target_angle =
//             vector3D(cos(l_angleMaintain), sin(l_angleMaintain), 0);
//         vector3D r_target_angle =
//             vector3D(cos(r_angleMaintain), sin(r_angleMaintain), 0);
//         vector3D l_current_angle = vector3D(cos(left_angle), sin(left_angle), 0);
//         vector3D r_current_angle = vector3D(cos(right_angle), sin(right_angle), 0);

//         l_error = angle(l_current_angle, l_target_angle);
//         r_error = angle(r_current_angle, r_target_angle);

//         // calculate the PID output
//         l_angle_pid = left_angle_PID.step(l_error);
//         r_angle_pid = right_angle_PID.step(r_error);
//         current_angle = imu.get_heading();
//         error = turn_target_angle - current_angle;
//         double derivative = error - turn_previous_error;
//         turn_previous_error = error;

//         double power = kP * error + kD * derivative;
//         pros::lcd::print(2, "Current Angle: %f", current_angle);
//         pros::lcd::print(3, "Error: %f", error);
//         pros::lcd::print(4, "Power: %f", power);

//         luA.move(power + l_angle_pid); // Adjust left side for turning
//         luB.move(power + l_angle_pid);
//         llA.move(power - l_angle_pid);
//         llA.move(power - l_angle_pid);
//         ruA.move(-power + r_angle_pid); // Adjust right side for turning
//         ruB.move(-power + r_angle_pid);
//         rlA.move(-power - r_angle_pid);
//         rlB.move(-power - r_angle_pid);
//         pros::delay(5);
//         if (fabs(error) < 2)
//         {

//             brake();
//             luA.move(0); // Adjust left side for turning
//             luB.move(0);
//             llA.move(0);
//             llA.move(0);
//             ruA.move(0); // Adjust right side for turning
//             ruB.move(0);
//             rlA.move(0);
//             rlB.move(0);
//             break;
//         }
//     }
// }

// void base_PID_front_back(double base_kp, double base_ki, double base_kd,
//                          double targetangle, double targetDistance_Y,
//                          double decelerationThreshold, double offset_kp, double offset_kd)
// {
//     // Movement variables
//     double powerL = 0;
//     double powerR = 0;
//     double errorLeft = 0;
//     double errorRight = 0;
//     double prevErrorLeft = 0;
//     double prevErrorRight = 0;
//     double totalErrorLeft = 0;
//     double totalErrorRight = 0;

//     double offset_error = 0;
//     double prev_offset_error = 0;
//     double offset_correction = 0;

//     double l_angleMaintain =
//         (targetangle - 90.0) *
//         TO_RADIANS; // note that the function getNormalizedSensorAngle already
//                     // implements wrapAngle to bound the angle between -180 and
//                     // 180 degrees
//     double r_angleMaintain =
//         (targetangle - 90.0) *
//         TO_RADIANS; // note that the function getNormalizedSensorAngle already
//                     // implements wrapAngle to bound the angle between -180 and
//                     // 180 degrees

//     double left_angle, right_angle;
//     double l_error = 0.0;
//     double r_error = 0.0;
//     // power output for angle component of pid
//     double l_angle_pid = 0.0;
//     double r_angle_pid = 0.0;
//     // 1. Perform turning first

//     bool l_move = true;
//     bool r_move = true;
//     int timeout = 0;

//     PID left_angle_PID(angle_kP, angle_kI, angle_kD);
//     PID right_angle_PID(angle_kP, angle_kI, angle_kD);

//     double aim_target = targetDistance_Y - global_distY;
//     original_x = global_distX;

//     while (l_move || r_move)
//     {

//         left_angle = (getNormalizedSensorAngle(left_rotation_sensor) - 90.0) *
//                      TO_RADIANS; // note that the function getNormalizedSensorAngle
//                                  // already implements wrapAngle to bound the angle
//                                  // between -180 and 180 degrees
//         right_angle =
//             (getNormalizedSensorAngle(right_rotation_sensor) - 90.0) * TO_RADIANS;

//         vector3D l_target_angle =
//             vector3D(cos(l_angleMaintain), sin(l_angleMaintain), 0);
//         vector3D r_target_angle =
//             vector3D(cos(r_angleMaintain), sin(r_angleMaintain), 0);
//         vector3D l_current_angle = vector3D(cos(left_angle), sin(left_angle), 0);
//         vector3D r_current_angle = vector3D(cos(right_angle), sin(right_angle), 0);

//         l_error = angle(l_current_angle, l_target_angle);
//         r_error = angle(r_current_angle, r_target_angle);

//         // calculate the PID output
//         l_angle_pid = left_angle_PID.step(l_error);
//         r_angle_pid = right_angle_PID.step(r_error);
//         // Calculate distance error

//         // double targetDistance = sqrt((pow ( fabs(targetDistance_X) -
//         // fabs(global_distX),2) +  pow (fabs(targetDistance_Y) -
//         // fabs(global_distY),2) ));
//         double targetDistance = aim_target - global_distY;
//         pros::lcd::print(
//             4, "current_distance: %.lf",
//             targetDistance);

//         errorLeft = targetDistance;
//         errorRight = targetDistance;
//         totalErrorLeft += errorLeft;
//         totalErrorRight += errorRight;

//         offset_error = global_distX - original_x;
//         float offset_derivative = offset_error - prev_offset_error;
//         prev_offset_error = offset_error;
//         offset_correction = (offset_kp * offset_error) + (offset_kd * offset_derivative);

//         // PID for left motors
//         if (fabs(errorLeft) <= base_error)
//         {
//             powerL = 0.0;
//             l_move = false;
//         }
//         else
//         {
//             // Gradually reduce power as you approach the target
//             if (fabs(errorLeft) < decelerationThreshold)
//             {
//                 powerL *= 0.5; // Reduce power to half when close to target
//             }
//             else
//             {
//                 powerL = base_kp * errorLeft + base_ki * totalErrorLeft +
//                          base_kd * (errorLeft - prevErrorLeft);
//             }
//             powerL = std::clamp(powerL, -200.0, 300.0);
//         }

//         // PID for right motors
//         if (fabs(errorRight) <= base_error)
//         {
//             powerR = 0.0;
//             r_move = false;
//         }
//         else
//         {
//             // Gradually reduce power as you approach the target
//             if (fabs(errorRight) < decelerationThreshold)
//             {
//                 powerR *= 0.5; // Reduce power to half when close to target
//             }
//             else
//             {
//                 powerR = base_kp * errorRight + base_ki * totalErrorRight +
//                          base_kd * (errorRight - prevErrorRight);
//             }
//             powerR = std::clamp(powerR, -200.0, 300.0);
//         }
//         pros::lcd::print(
//             5, "powerL: %.lf",
//             powerL);
//         pros::lcd::print(
//             6, "powerR: %.lf",
//             powerR);
//         // Move the motors
//         if (targetDistance_Y >= 0)
//         {
//             pros::lcd::print(
//                 0, "targetDistance_Y: %.lf",
//                 targetDistance_Y);
//             luA.move_velocity(powerL + l_angle_pid);
//             luB.move_velocity(powerL + l_angle_pid);
//             llA.move_velocity(powerL - l_angle_pid);
//             llB.move_velocity(powerL - l_angle_pid);
//             ruA.move_velocity(powerR + r_angle_pid);
//             ruB.move_velocity(powerR + r_angle_pid);
//             rlA.move_velocity(powerR - r_angle_pid);
//             rlB.move_velocity(powerR - r_angle_pid);
//             if (errorRight < 0 && errorLeft < 0)
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
//             luA.move_velocity(powerL + l_angle_pid);
//             luB.move_velocity(powerL + l_angle_pid);
//             llA.move_velocity(powerL - l_angle_pid);
//             llB.move_velocity(powerL - l_angle_pid);
//             ruA.move_velocity(powerR + r_angle_pid);
//             ruB.move_velocity(powerR + r_angle_pid);
//             rlA.move_velocity(powerR - r_angle_pid);
//             rlB.move_velocity(powerR - r_angle_pid);
//             if (errorRight > 0 && errorLeft > 0)
//             {
//                 brake();
//                 break;
//             }
//         }

//         if (timeout >= 100000)
//             break;

//         if (fabs(powerR) < 2 || fabs(powerL) < 2)
//         {
//             break;
//         }

//         // pros::lcd::print(0, "ErrorL: %.lf", errorLeft);
//         // pros::lcd::print(1, "ErrorR: %.lf", errorRight);

//         // Update previous errors for the next iteration
//         prevErrorLeft = errorLeft;
//         prevErrorRight = errorRight;

//         pros::delay(2); // Delay to reduce CPU load
//         timeout += 2;
//     }
//     brake();
// }

// void base_PID_front_back_flipped(double base_kp_left, double base_kd_left, double base_kp_right,double base_kd_right,
//                          double targetangle, double targetDistance_Y,
//                          double decelerationThreshold, double offset_kp, double offset_kd)
// {
//     // Movement variables
//     double powerL = 0;
//     double powerR = 0;
//     double errorLeft = 0;
//     double errorRight = 0;
//     double prevErrorLeft = 0;
//     double prevErrorRight = 0;
//     double totalErrorLeft = 0;
//     double totalErrorRight = 0;

//     double offset_error = 0;
//     double prev_offset_error = 0;
//     double offset_correction = 0;

//     double l_angleMaintain =
//         (targetangle - 90.0) *
//         TO_RADIANS; // note that the function getNormalizedSensorAngle already
//                     // implements wrapAngle to bound the angle between -180 and
//                     // 180 degrees
//     double r_angleMaintain =
//         (targetangle - 90.0) *
//         TO_RADIANS; // note that the function getNormalizedSensorAngle already
//                     // implements wrapAngle to bound the angle between -180 and
//                     // 180 degrees

//     double left_angle, right_angle;
//     double l_error = 0.0;
//     double r_error = 0.0;
//     // power output for angle component of pid
//     double l_angle_pid = 0.0;
//     double r_angle_pid = 0.0;
//     // 1. Perform turning first

//     bool l_move = true;
//     bool r_move = true;
//     int timeout = 0;

//     PID left_angle_PID(angle_kP, angle_kI, angle_kD);
//     PID right_angle_PID(angle_kP, angle_kI, angle_kD);

//     double aim_target = targetDistance_Y - global_distY;
//     original_x = global_distX;

//     while (l_move || r_move)
//     {

//         left_angle = (getNormalizedSensorAngle(left_rotation_sensor) - 90.0) *
//                      TO_RADIANS; // note that the function getNormalizedSensorAngle
//                                  // already implements wrapAngle to bound the angle
//                                  // between -180 and 180 degrees
//         right_angle =
//             (getNormalizedSensorAngle(right_rotation_sensor) - 90.0) * TO_RADIANS;

//         vector3D l_target_angle =
//             vector3D(cos(l_angleMaintain), sin(l_angleMaintain), 0);
//         vector3D r_target_angle =
//             vector3D(cos(r_angleMaintain), sin(r_angleMaintain), 0);
//         vector3D l_current_angle = vector3D(cos(left_angle), sin(left_angle), 0);
//         vector3D r_current_angle = vector3D(cos(right_angle), sin(right_angle), 0);

//         l_error = angle(l_current_angle, l_target_angle);
//         r_error = angle(r_current_angle, r_target_angle);

//         // calculate the PID output
//         l_angle_pid = left_angle_PID.step(l_error);
//         r_angle_pid = right_angle_PID.step(r_error);
//         // Calculate distance error

//         // double targetDistance = sqrt((pow ( fabs(targetDistance_X) -
//         // fabs(global_distX),2) +  pow (fabs(targetDistance_Y) -
//         // fabs(global_distY),2) ));
//         double targetDistance = aim_target - global_distY;
//         pros::lcd::print(
//             4, "current_distance: %.lf",
//             targetDistance);

//         errorLeft = targetDistance;
//         errorRight = targetDistance;
//         totalErrorLeft += errorLeft;
//         totalErrorRight += errorRight;

//         offset_error = global_distX - original_x;
//         float offset_derivative = offset_error - prev_offset_error;
//         prev_offset_error = offset_error;
//         offset_correction = (offset_kp * offset_error) + (offset_kd * offset_derivative);

//         // PID for left motors
//         if (fabs(errorLeft) <= base_error)
//         {
//             powerL = 0.0;
//             l_move = false;
//         }
//         else
//         {
//             // Gradually reduce power as you approach the target
//             if (fabs(errorLeft) < decelerationThreshold)
//             {
//                 powerL *= 0.5; // Reduce power to half when close to target
//             }
//             else
//             {
//                 powerL = base_kp_left * errorLeft +
//                          base_kd_left * (errorLeft - prevErrorLeft);
//             }
//             powerL = std::clamp(powerL, -200.0, 300.0);
//         }

//         // PID for right motors
//         if (fabs(errorRight) <= base_error)
//         {
//             powerR = 0.0;
//             r_move = false;
//         }
//         else
//         {
//             // Gradually reduce power as you approach the target
//             if (fabs(errorRight) < decelerationThreshold)
//             {
//                 powerR *= 0.5; // Reduce power to half when close to target
//             }
//             else
//             {
//                 powerR = base_kp_right * errorRight +
//                          base_kd_right * (errorRight - prevErrorRight);
//             }
//             powerR = std::clamp(powerR, -200.0, 300.0);
//         }
//         pros::lcd::print(
//             5, "powerL: %.lf",
//             powerL);
//         pros::lcd::print(
//             6, "powerR: %.lf",
//             powerR);
//         // Move the motors
//         if (targetDistance_Y >= 0)
//         {
//             pros::lcd::print(
//                 0, "targetDistance_Y: %.lf",
//                 targetDistance_Y);
//             luA.move_velocity(-powerL + l_angle_pid);
//             luB.move_velocity(-powerL + l_angle_pid);
//             llA.move_velocity(-powerL - l_angle_pid);
//             llB.move_velocity(-powerL - l_angle_pid);
//             ruA.move_velocity(-powerR + r_angle_pid);
//             ruB.move_velocity(-powerR + r_angle_pid);
//             rlA.move_velocity(-powerR - r_angle_pid);
//             rlB.move_velocity(-powerR - r_angle_pid);
//             if (errorRight < 0 && errorLeft < 0)
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
//             luA.move_velocity(-powerL + l_angle_pid);
//             luB.move_velocity(-powerL + l_angle_pid);
//             llA.move_velocity(-powerL - l_angle_pid);
//             llB.move_velocity(-powerL - l_angle_pid);
//             ruA.move_velocity(-powerR + r_angle_pid);
//             ruB.move_velocity(-powerR + r_angle_pid);
//             rlA.move_velocity(-powerR - r_angle_pid);
//             rlB.move_velocity(-powerR - r_angle_pid);
//             if (errorRight > 0 && errorLeft > 0)
//             {
//                 brake();
//                 break;
//             }
//         }

//         if (timeout >= 100000)
//             break;

//         if (fabs(powerR) < 2 || fabs(powerL) < 2)
//         {
//             break;
//         }

//         // pros::lcd::print(0, "ErrorL: %.lf", errorLeft);
//         // pros::lcd::print(1, "ErrorR: %.lf", errorRight);

//         // Update previous errors for the next iteration
//         prevErrorLeft = errorLeft;
//         prevErrorRight = errorRight;

//         pros::delay(2); // Delay to reduce CPU load
//         timeout += 2;
//     }
//     brake();
// }

// void base_PID_left_right(double base_kp, double base_ki, double base_kd,
//                          double targetangle, double targetDistance_X = 0,
//                          double decelerationThreshold = 0)
// {
//     // Movement variables
//     double powerL = 0;
//     double powerR = 0;
//     double encdleft = 0;
//     double encdright = 0;
//     double errorLeft = 0;
//     double errorRight = 0;
//     double prevErrorLeft = 0;
//     double prevErrorRight = 0;
//     double totalErrorLeft = 0;
//     double totalErrorRight = 0;

//     double l_angleMaintain =
//         (targetangle - 90.0) *
//         TO_RADIANS; // note that the function getNormalizedSensorAngle already
//                     // implements wrapAngle to bound the angle between -180 and
//                     // 180 degrees
//     double r_angleMaintain =
//         (targetangle - 90.0) *
//         TO_RADIANS; // note that the function getNormalizedSensorAngle already
//                     // implements wrapAngle to bound the angle between -180 and
//                     // 180 degrees

//     double left_angle, right_angle;
//     double l_error = 0.0;
//     double r_error = 0.0;
//     // power output for angle component of pid
//     double l_angle_pid = 0.0;
//     double r_angle_pid = 0.0;
//     // 1. Perform turning first

//     bool l_move = true;
//     bool r_move = true;
//     int timeout = 0;

//     PID left_angle_PID(angle_kP_mehmeh, angle_kI, angle_kD);
//     PID right_angle_PID(angle_kP_mehmeh, angle_kI, angle_kD);

//     double aim_target = targetDistance_X - global_distX;
//     double original_y = global_distY;

//     while (l_move || r_move)
//     {

//         left_angle = (getNormalizedSensorAngle(left_rotation_sensor) - 90.0) *
//                      TO_RADIANS; // note that the function getNormalizedSensorAngle
//                                  // already implements wrapAngle to bound the angle
//                                  // between -180 and 180 degrees
//         right_angle =
//             (getNormalizedSensorAngle(right_rotation_sensor) - 90.0) * TO_RADIANS;

//         vector3D l_target_angle =
//             vector3D(cos(l_angleMaintain), sin(l_angleMaintain), 0);
//         vector3D r_target_angle =
//             vector3D(cos(r_angleMaintain), sin(r_angleMaintain), 0);
//         vector3D l_current_angle = vector3D(cos(left_angle), sin(left_angle), 0);
//         vector3D r_current_angle = vector3D(cos(right_angle), sin(right_angle), 0);

//         l_error = angle(l_current_angle, l_target_angle);
//         r_error = angle(r_current_angle, r_target_angle);

//         // calculate the PID output
//         l_angle_pid = left_angle_PID.step(l_error);
//         r_angle_pid = right_angle_PID.step(r_error);
//         // Calculate distance error

//         double targetDistance = aim_target - global_distX;
//         pros::lcd::print(
//             4, "l_angle_pid: %.lf",
//             l_angle_pid);

//         errorLeft = targetDistance;
//         errorRight = targetDistance;
//         totalErrorLeft += errorLeft;
//         totalErrorRight += errorRight;

//         // PID for left motors
//         if (fabs(errorLeft) <= base_error)
//         {
//             powerL = 0.0;
//             l_move = false;
//         }
//         else
//         {
//             // Gradually reduce power as you approach the target
//             if (fabs(errorLeft) < decelerationThreshold)
//             {
//                 powerL *= 0.5; // Reduce power to half when close to target
//             }
//             else
//             {
//                 powerL = base_kp * errorLeft + base_ki * totalErrorLeft +
//                          base_kd * (errorLeft - prevErrorLeft);
//             }
//             powerL = std::clamp(powerL, -200.0, 300.0);
//         }

//         // PID for right motors
//         if (fabs(errorRight) <= base_error)
//         {
//             powerR = 0.0;
//             r_move = false;
//         }
//         else
//         {
//             // Gradually reduce power as you approach the target
//             if (fabs(errorRight) < decelerationThreshold)
//             {
//                 powerR *= 0.5; // Reduce power to half when close to target
//             }
//             else
//             {
//                 powerR = base_kp * errorRight + base_ki * totalErrorRight +
//                          base_kd * (errorRight - prevErrorRight);
//             }
//             powerR = std::clamp(powerR, -200.0, 300.0);
//         }
//         pros::lcd::print(
//             5, "powerL: %.lf",
//             powerL);
//         pros::lcd::print(
//             6, "powerR: %.lf",
//             powerR);
//         // Move the motors

//         if (fabs(powerR) < 2 || fabs(powerL) < 2)
//         {
//             break;
//         }

//         if (targetDistance_X >= 0)
//         {

//             // luA.move_velocity(powerL - l_angle_pid);
//             // luB.move_velocity(powerL - l_angle_pid);
//             // llA.move_velocity(powerL + l_angle_pid);
//             // llB.move_velocity(powerL + l_angle_pid);
//             // ruA.move_velocity(powerR - r_angle_pid);
//             // ruB.move_velocity(powerR - r_angle_pid);
//             // rlA.move_velocity(powerR + r_angle_pid);
//             // rlB.move_velocity(powerR + r_angle_pid);

//             // luA.move_velocity(powerL + l_angle_pid);
//             // luB.move_velocity(powerL + l_angle_pid);
//             // llA.move_velocity(powerL - l_angle_pid);
//             // llB.move_velocity(powerL - l_angle_pid);
//             // ruA.move_velocity(powerR + r_angle_pid);
//             // ruB.move_velocity(powerR + r_angle_pid);
//             // rlA.move_velocity(powerR - r_angle_pid);
//             // rlB.move_velocity(powerR - r_angle_pid);

//             // luA.move_velocity(-powerL - l_angle_pid);
//             // luB.move_velocity(-powerL - l_angle_pid);
//             // llA.move_velocity(-powerL + l_angle_pid);
//             // llB.move_velocity(-powerL + l_angle_pid);
//             // ruA.move_velocity(-powerR - r_angle_pid);
//             // ruB.move_velocity(-powerR - r_angle_pid);
//             // rlA.move_velocity(-powerR + r_angle_pid);
//             // rlB.move_velocity(-powerR + r_angle_pid);

//             luA.move_velocity(-powerL + l_angle_pid);
//             luB.move_velocity(-powerL + l_angle_pid);
//             llA.move_velocity(-powerL - l_angle_pid);
//             llB.move_velocity(-powerL - l_angle_pid);
//             ruA.move_velocity(-powerR + r_angle_pid);
//             ruB.move_velocity(-powerR + r_angle_pid);
//             rlA.move_velocity(-powerR - r_angle_pid);
//             rlB.move_velocity(-powerR - r_angle_pid);

//             if (errorRight < 0 && errorLeft < 0)

//             // if (errorRight > 0 && errorLeft > 0)
//             {
//                 brake();
//                 break;
//             }
//         }
//         else
//         {
//             // luA.move_velocity(powerL - l_angle_pid);
//             // luB.move_velocity(powerL - l_angle_pid);
//             // llA.move_velocity(powerL + l_angle_pid);
//             // llB.move_velocity(powerL + l_angle_pid);
//             // ruA.move_velocity(powerR - r_angle_pid);
//             // ruB.move_velocity(powerR - r_angle_pid);
//             // rlA.move_velocity(powerR + r_angle_pid);
//             // rlB.move_velocity(powerR + r_angle_pid);

//             // luA.move_velocity(powerL + l_angle_pid);
//             // luB.move_velocity(powerL + l_angle_pid);
//             // llA.move_velocity(powerL - l_angle_pid);
//             // llB.move_velocity(powerL - l_angle_pid);
//             // ruA.move_velocity(powerR + r_angle_pid);
//             // ruB.move_velocity(powerR + r_angle_pid);
//             // rlA.move_velocity(powerR - r_angle_pid);
//             // rlB.move_velocity(powerR - r_angle_pid);

//             luA.move_velocity(-powerL + l_angle_pid);
//             luB.move_velocity(-powerL + l_angle_pid);
//             llA.move_velocity(-powerL - l_angle_pid);
//             llB.move_velocity(-powerL - l_angle_pid);
//             ruA.move_velocity(-powerR + r_angle_pid);
//             ruB.move_velocity(-powerR + r_angle_pid);
//             rlA.move_velocity(-powerR - r_angle_pid);
//             rlB.move_velocity(-powerR - r_angle_pid);

//             // luA.move_velocity(-powerL - l_angle_pid);
//             // luB.move_velocity(-powerL - l_angle_pid);
//             // llA.move_velocity(-powerL + l_angle_pid);
//             // llB.move_velocity(-powerL + l_angle_pid);
//             // ruA.move_velocity(-powerR - r_angle_pid);
//             // ruB.move_velocity(-powerR - r_angle_pid);
//             // rlA.move_velocity(-powerR + r_angle_pid);
//             // rlB.move_velocity(-powerR + r_angle_pid);

//             // if (errorRight < 0 && errorLeft < 0)
//             if (errorRight > 0 && errorLeft > 0)
//             {
//                 brake();
//                 break;
//             }
//         }

//         if (timeout >= 100000)
//             break;

//         pros::lcd::print(0, "ErrorL: %.lf", errorLeft);
//         pros::lcd::print(1, "ErrorR: %.lf", errorRight);

//         // Update previous errors for the next iteration
//         prevErrorLeft = errorLeft;
//         prevErrorRight = errorRight;

//         pros::delay(2); // Delay to reduce CPU load
//         timeout += 2;
//     }
//     brake();
// }

// void base_PID_left_right_flipped(double base_kp, double base_ki, double base_kd,
//                          double targetangle, double targetDistance_X = 0,
//                          double decelerationThreshold = 0)
// {
//     // Movement variables
//     double powerL = 0;
//     double powerR = 0;
//     double encdleft = 0;
//     double encdright = 0;
//     double errorLeft = 0;
//     double errorRight = 0;
//     double prevErrorLeft = 0;
//     double prevErrorRight = 0;
//     double totalErrorLeft = 0;
//     double totalErrorRight = 0;

//     double l_angleMaintain =
//         (targetangle - 90.0) *
//         TO_RADIANS; // note that the function getNormalizedSensorAngle already
//                     // implements wrapAngle to bound the angle between -180 and
//                     // 180 degrees
//     double r_angleMaintain =
//         (targetangle - 90.0) *
//         TO_RADIANS; // note that the function getNormalizedSensorAngle already
//                     // implements wrapAngle to bound the angle between -180 and
//                     // 180 degrees

//     double left_angle, right_angle;
//     double l_error = 0.0;
//     double r_error = 0.0;
//     // power output for angle component of pid
//     double l_angle_pid = 0.0;
//     double r_angle_pid = 0.0;
//     // 1. Perform turning first

//     bool l_move = true;
//     bool r_move = true;
//     int timeout = 0;

//     PID left_angle_PID(angle_kP_mehmeh, angle_kI, angle_kD);
//     PID right_angle_PID(angle_kP_mehmeh, angle_kI, angle_kD);

//     double aim_target = targetDistance_X - global_distX;
//     double original_y = global_distY;

//     while (l_move || r_move)
//     {

//         left_angle = (getNormalizedSensorAngle(left_rotation_sensor) - 90.0) *
//                      TO_RADIANS; // note that the function getNormalizedSensorAngle
//                                  // already implements wrapAngle to bound the angle
//                                  // between -180 and 180 degrees
//         right_angle =
//             (getNormalizedSensorAngle(right_rotation_sensor) - 90.0) * TO_RADIANS;

//         vector3D l_target_angle =
//             vector3D(cos(l_angleMaintain), sin(l_angleMaintain), 0);
//         vector3D r_target_angle =
//             vector3D(cos(r_angleMaintain), sin(r_angleMaintain), 0);
//         vector3D l_current_angle = vector3D(cos(left_angle), sin(left_angle), 0);
//         vector3D r_current_angle = vector3D(cos(right_angle), sin(right_angle), 0);

//         l_error = angle(l_current_angle, l_target_angle);
//         r_error = angle(r_current_angle, r_target_angle);

//         // calculate the PID output
//         l_angle_pid = left_angle_PID.step(l_error);
//         r_angle_pid = right_angle_PID.step(r_error);
//         // Calculate distance error

//         double targetDistance = aim_target - global_distX;
//         pros::lcd::print(
//             4, "l_angle_pid: %.lf",
//             l_angle_pid);

//         errorLeft = targetDistance;
//         errorRight = targetDistance;
//         totalErrorLeft += errorLeft;
//         totalErrorRight += errorRight;

//         // PID for left motors
//         if (fabs(errorLeft) <= base_error)
//         {
//             powerL = 0.0;
//             l_move = false;
//         }
//         else
//         {
//             // Gradually reduce power as you approach the target
//             if (fabs(errorLeft) < decelerationThreshold)
//             {
//                 powerL *= 0.5; // Reduce power to half when close to target
//             }
//             else
//             {
//                 powerL = base_kp * errorLeft + base_ki * totalErrorLeft +
//                          base_kd * (errorLeft - prevErrorLeft);
//             }
//             powerL = std::clamp(powerL, -200.0, 300.0);
//         }

//         // PID for right motors
//         if (fabs(errorRight) <= base_error)
//         {
//             powerR = 0.0;
//             r_move = false;
//         }
//         else
//         {
//             // Gradually reduce power as you approach the target
//             if (fabs(errorRight) < decelerationThreshold)
//             {
//                 powerR *= 0.5; // Reduce power to half when close to target
//             }
//             else
//             {
//                 powerR = base_kp * errorRight + base_ki * totalErrorRight +
//                          base_kd * (errorRight - prevErrorRight);
//             }
//             powerR = std::clamp(powerR, -200.0, 300.0);
//         }
//         pros::lcd::print(
//             5, "powerL: %.lf",
//             powerL);
//         pros::lcd::print(
//             6, "powerR: %.lf",
//             powerR);
//         // Move the motors

//         if (fabs(powerR) < 2 || fabs(powerL) < 2)
//         {
//             break;
//         }

//         if (targetDistance_X >= 0)
//         {

//             // luA.move_velocity(powerL - l_angle_pid);
//             // luB.move_velocity(powerL - l_angle_pid);
//             // llA.move_velocity(powerL + l_angle_pid);
//             // llB.move_velocity(powerL + l_angle_pid);
//             // ruA.move_velocity(powerR - r_angle_pid);
//             // ruB.move_velocity(powerR - r_angle_pid);
//             // rlA.move_velocity(powerR + r_angle_pid);
//             // rlB.move_velocity(powerR + r_angle_pid);

//             // luA.move_velocity(powerL + l_angle_pid);
//             // luB.move_velocity(powerL + l_angle_pid);
//             // llA.move_velocity(powerL - l_angle_pid);
//             // llB.move_velocity(powerL - l_angle_pid);
//             // ruA.move_velocity(powerR + r_angle_pid);
//             // ruB.move_velocity(powerR + r_angle_pid);
//             // rlA.move_velocity(powerR - r_angle_pid);
//             // rlB.move_velocity(powerR - r_angle_pid);

//             // luA.move_velocity(-powerL - l_angle_pid);
//             // luB.move_velocity(-powerL - l_angle_pid);
//             // llA.move_velocity(-powerL + l_angle_pid);
//             // llB.move_velocity(-powerL + l_angle_pid);
//             // ruA.move_velocity(-powerR - r_angle_pid);
//             // ruB.move_velocity(-powerR - r_angle_pid);
//             // rlA.move_velocity(-powerR + r_angle_pid);
//             // rlB.move_velocity(-powerR + r_angle_pid);

//             luA.move_velocity(powerL + l_angle_pid);
//             luB.move_velocity(powerL + l_angle_pid);
//             llA.move_velocity(powerL - l_angle_pid);
//             llB.move_velocity(powerL - l_angle_pid);
//             ruA.move_velocity(powerR + r_angle_pid);
//             ruB.move_velocity(powerR + r_angle_pid);
//             rlA.move_velocity(powerR - r_angle_pid);
//             rlB.move_velocity(powerR - r_angle_pid);

//             if (errorRight < 0 && errorLeft < 0)

//             // if (errorRight > 0 && errorLeft > 0)
//             {
//                 brake();
//                 break;
//             }
//         }
//         else
//         {
//             // luA.move_velocity(powerL - l_angle_pid);
//             // luB.move_velocity(powerL - l_angle_pid);
//             // llA.move_velocity(powerL + l_angle_pid);
//             // llB.move_velocity(powerL + l_angle_pid);
//             // ruA.move_velocity(powerR - r_angle_pid);
//             // ruB.move_velocity(powerR - r_angle_pid);
//             // rlA.move_velocity(powerR + r_angle_pid);
//             // rlB.move_velocity(powerR + r_angle_pid);

//             // luA.move_velocity(powerL + l_angle_pid);
//             // luB.move_velocity(powerL + l_angle_pid);
//             // llA.move_velocity(powerL - l_angle_pid);
//             // llB.move_velocity(powerL - l_angle_pid);
//             // ruA.move_velocity(powerR + r_angle_pid);
//             // ruB.move_velocity(powerR + r_angle_pid);
//             // rlA.move_velocity(powerR - r_angle_pid);
//             // rlB.move_velocity(powerR - r_angle_pid);

//             luA.move_velocity(powerL + l_angle_pid);
//             luB.move_velocity(powerL + l_angle_pid);
//             llA.move_velocity(powerL - l_angle_pid);
//             llB.move_velocity(powerL - l_angle_pid);
//             ruA.move_velocity(powerR + r_angle_pid);
//             ruB.move_velocity(powerR + r_angle_pid);
//             rlA.move_velocity(powerR - r_angle_pid);
//             rlB.move_velocity(powerR - r_angle_pid);

//             // luA.move_velocity(-powerL - l_angle_pid);
//             // luB.move_velocity(-powerL - l_angle_pid);
//             // llA.move_velocity(-powerL + l_angle_pid);
//             // llB.move_velocity(-powerL + l_angle_pid);
//             // ruA.move_velocity(-powerR - r_angle_pid);
//             // ruB.move_velocity(-powerR - r_angle_pid);
//             // rlA.move_velocity(-powerR + r_angle_pid);
//             // rlB.move_velocity(-powerR + r_angle_pid);

//             // if (errorRight < 0 && errorLeft < 0)
//             if (errorRight > 0 && errorLeft > 0)
//             {
//                 brake();
//                 break;
//             }
//         }

//         if (timeout >= 100000)
//             break;

//         pros::lcd::print(0, "ErrorL: %.lf", errorLeft);
//         pros::lcd::print(1, "ErrorR: %.lf", errorRight);

//         // Update previous errors for the next iteration
//         prevErrorLeft = errorLeft;
//         prevErrorRight = errorRight;

//         pros::delay(2); // Delay to reduce CPU load
//         timeout += 2;
//     }
//     brake();
// }


// void base_PID_side(double base_kp, double base_ki, double base_kd,
//                    double targetangle, double targetDistance,
//                    double decelerationThreshold = 0)
// {
//     // Movement variables
//     double powerL = 0;
//     double powerR = 0;
//     double errorLeft = 0;
//     double errorRight = 0;
//     double prevErrorLeft = 0;
//     double prevErrorRight = 0;
//     double totalErrorLeft = 0;
//     double totalErrorRight = 0;

//     double l_angleMaintain =
//         (targetangle - 90.0) *
//         TO_RADIANS; // note that the function getNormalizedSensorAngle already
//                     // implements wrapAngle to bound the angle between -180 and
//                     // 180 degrees
//     double r_angleMaintain =
//         (targetangle - 90.0) *
//         TO_RADIANS; // note that the function getNormalizedSensorAngle already
//                     // implements wrapAngle to bound the angle between -180 and
//                     // 180 degrees

//     double left_angle, right_angle;
//     double l_error = 0.0;
//     double r_error = 0.0;
//     // power output for angle component of pid
//     double l_angle_pid = 0.0;
//     double r_angle_pid = 0.0;
//     // 1. Perform turning first

//     bool l_move = true;
//     bool r_move = true;
//     int timeout = 0;

//     PID left_angle_PID(angle_kP, angle_kI, angle_kD);
//     PID right_angle_PID(angle_kP, angle_kI, angle_kD);

//     double aim_target = targetDistance;
//     double original_y = global_distY;

//     while (l_move || r_move)
//     {

//         left_angle = (getNormalizedSensorAngle(left_rotation_sensor) - 90.0) *
//                      TO_RADIANS; // note that the function getNormalizedSensorAngle
//                                  // already implements wrapAngle to bound the angle
//                                  // between -180 and 180 degrees
//         right_angle =
//             (getNormalizedSensorAngle(right_rotation_sensor) - 90.0) * TO_RADIANS;

//         vector3D l_target_angle =
//             vector3D(cos(l_angleMaintain), sin(l_angleMaintain), 0);
//         vector3D r_target_angle =
//             vector3D(cos(r_angleMaintain), sin(r_angleMaintain), 0);
//         vector3D l_current_angle = vector3D(cos(left_angle), sin(left_angle), 0);
//         vector3D r_current_angle = vector3D(cos(right_angle), sin(right_angle), 0);

//         l_error = angle(l_current_angle, l_target_angle);
//         r_error = angle(r_current_angle, r_target_angle);

//         // calculate the PID output
//         l_angle_pid = left_angle_PID.step(l_error);
//         r_angle_pid = right_angle_PID.step(r_error);
//         // Calculate distance error

//         double targetDistance;
//         // Calculate distance error
//         if (aim_target > 0)
//         {
//             double targetDistance = aim_target - sqrt((pow(
//                                                            fabs(global_distX), 2) +
//                                                        pow(fabs(global_distY), 2)));
//         }
//         else
//         {
//             double targetDistance = aim_target + sqrt((pow(
//                                                            fabs(global_distX), 2) +
//                                                        pow(fabs(global_distY), 2)));
//         }

//         // double targetDistance = aim_target - global_distX;
//         pros::lcd::print(
//             4, "current_distance: %.lf",
//             targetDistance);
//         pros::lcd::print(
//             4, "current_distance: %.lf",
//             targetDistance);

//         errorLeft = targetDistance;
//         errorRight = targetDistance;
//         totalErrorLeft += errorLeft;
//         totalErrorRight += errorRight;

//         // PID for left motors
//         if (fabs(errorLeft) <= base_error)
//         {
//             powerL = 0.0;
//             l_move = false;
//         }
//         else
//         {
//             // Gradually reduce power as you approach the target
//             if (fabs(errorLeft) < decelerationThreshold)
//             {
//                 powerL *= 0.5; // Reduce power to half when close to target
//             }
//             else
//             {
//                 powerL = base_kp * errorLeft + base_ki * totalErrorLeft +
//                          base_kd * (errorLeft - prevErrorLeft);
//             }
//             powerL = std::clamp(powerL, 50.0, 300.0);
//         }
//         pros::lcd::print(
//             5, "powerL: %.lf",
//             powerL);

//         // PID for right motors
//         if (fabs(errorRight) <= base_error)
//         {
//             powerR = 0.0;
//             r_move = false;
//         }
//         else
//         {
//             // Gradually reduce power as you approach the target
//             if (fabs(errorRight) < decelerationThreshold)
//             {
//                 powerR *= 0.5; // Reduce power to half when close to target
//             }
//             else
//             {
//                 powerR = base_kp * errorRight + base_ki * totalErrorRight +
//                          base_kd * (errorRight - prevErrorRight);
//             }
//             powerR = std::clamp(powerR, 50.0, 300.0);
//         }
//         pros::lcd::print(
//             5, "l_angle_pid: %.lf",
//             l_angle_pid);
//         pros::lcd::print(
//             6, "r_angle_pid: %.lf",
//             r_angle_pid);
//         // Move the motors
//         if (targetDistance >= 0)
//         {

//             luA.move_velocity(powerL + l_angle_pid);
//             luB.move_velocity(powerL + l_angle_pid);
//             llA.move_velocity(powerL - l_angle_pid);
//             llB.move_velocity(powerL - l_angle_pid);
//             ruA.move_velocity(powerR + r_angle_pid);
//             ruB.move_velocity(powerR + r_angle_pid);
//             rlA.move_velocity(powerR - r_angle_pid);
//             rlB.move_velocity(powerR - r_angle_pid);
//             if (errorRight < 0 && errorLeft < 0)
//             {
//                 brake();
//                 break;
//             }
//         }
//         else
//         {

//             luA.move_velocity(-powerL - l_angle_pid);
//             luB.move_velocity(-powerL - l_angle_pid);
//             llA.move_velocity(-powerL + l_angle_pid);
//             llB.move_velocity(-powerL + l_angle_pid);
//             ruA.move_velocity(-powerR - r_angle_pid);
//             ruB.move_velocity(-powerR - r_angle_pid);
//             rlA.move_velocity(-powerR + r_angle_pid);
//             rlB.move_velocity(-powerR + r_angle_pid);
//             if (errorRight > 0 && errorLeft > 0)
//             {
//                 brake();
//                 break;
//             }
//         }

//         if (timeout >= 100000)
//             break;

//         pros::lcd::print(0, "ErrorL: %.lf", errorLeft);
//         pros::lcd::print(1, "ErrorR: %.lf", errorRight);

//         // Update previous errors for the next iteration
//         prevErrorLeft = errorLeft;
//         prevErrorRight = errorRight;

//         pros::delay(2); // Delay to reduce CPU load
//         timeout += 2;
//     }
//     brake();
// }

// void initialize()
// {
//     pros::lcd::initialize();

//     luA.set_brake_mode(MOTOR_BRAKE_COAST);
//     luB.set_brake_mode(MOTOR_BRAKE_COAST);
//     llA.set_brake_mode(MOTOR_BRAKE_COAST);
//     llB.set_brake_mode(MOTOR_BRAKE_COAST);
//     ruA.set_brake_mode(MOTOR_BRAKE_COAST);
//     ruB.set_brake_mode(MOTOR_BRAKE_COAST);
//     rlA.set_brake_mode(MOTOR_BRAKE_COAST);
//     rlB.set_brake_mode(MOTOR_BRAKE_COAST);
//     // liftL.set_brake_mode(MOTOR_BRAKE_HOLD);
//     // liftR.set_brake_mode(MOTOR_BRAKE_HOLD);

//     // while(!left_rotation_sensor.reset());
//     // while(!right_rotation_sensor.reset());

//     left_rotation_sensor.set_data_rate(5);
//     right_rotation_sensor.set_data_rate(5);

//     left_rotation_sensor.set_position(0);
//     right_rotation_sensor.set_position(0);

//     conveyor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
//     roller.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

//     // pros::Task move_base(moveBase);
//     pros::Task serial_read(serialRead);

//     master.clear();
// }

// void autonomous()
// {

//     set_wheel_angle(0, 0.5, 0.00, 0.1);

//     // base_PID_front_back(1.5, 0, 0,  0, -900,295, 0.5, 0.0);
//     base_PID_front_back(1.5, 0, 0, 0, 860, 250, 0.5, 0.0);

//     set_wheel_angle(-90, 1.5, 0.00, 0.4);

//     base_PID_left_right(1, 0, 0, 90, -300, 220);
//     set_wheel_angle(1, 1.5, 0.00, 0.4);

//     // base_PID(0.025, 0, 0, false, 0, 700,100);
//     // set_wheel_angle(0, 0.5, 0.00, 0.1);
//     // base_PID(700,5.0,0,0.2 );
// }

// void autonomousb()
// {
//           set_wheel_angle(0, 0.5, 0.00, 0.1);

//     //   base_PID_front_back_flipped(1.5, 0,1.7, 0.1, 0, 700, 400, 0.5, 0.0);
//         //   base_PID_front_back_flipped(1.35, 0,1.5, 0.5, 0, 860, 250, 0.5, 0.0);
//           base_PID_front_back_flipped(1.5, 0,1.5, 0, 0, 860, 250, 0.5, 0.0);
//           solenoid.set_value(0);
//         pros::Task::delay(100);
//         mobilegoal_bot.set_value(0);
//     // set_wheel_angle(-90, 0.05, 0.00, 0.1);
//     // base_PID_left_right(1, 0, 0, 90, 300, 220);
//     // set_wheel_angle(-90, 0.05, 0.00, 0.1);

// }

// void autonomousy()
// {

//     update_turning(180, 1.0, 0.1);
//     // Control update rate

//     //   set_wheel_angle(-90, 1.5, 0.00, 0.4);

//     //   base_PID_left_right(1, 0, 0, 90, -300, 230);
//     //   set_wheel_angle(-90, 0.05, 0.00, 0.1);
//     //   // base_PID(1000, 0, 0, false, 0, 500,400);

//     //   // base_PID(0.025, 0, 0, false, 0, -700,100);
//     //   set_wheel_angle(0, 0.5, 0.00, 0.1);
//     //   base_PID(700,5.0,0,0.2 );
// }

// void opcontrol()
// {
//     while (true)
//     {
//         if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X))
//         {
//             autonomous();
//         }
//         if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B))
//         {
//             autonomousb();
//         }
//         if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
//         {
//             autonomousy();
//         }
//         if (master.get_digital_new_press(DIGITAL_A))
//             mobile_goal_actuated = !mobile_goal_actuated;
//         if (mobile_goal_actuated)
//         {
//             solenoid.set_value(1);

//             mobilegoal_bot.set_value(0);
//         }

//         else
//         {

//             solenoid.set_value(0);
//             pros::Task::delay(100);
//             mobilegoal_bot.set_value(1);
//         }
//         pros::delay(5);
//     }
// }
