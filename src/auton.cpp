// #include "main.h"

// void disabled() {}
// void competition_initialize() {}

// // Function to determine sign of a integer variable, returns bool
// template <typename T> int sgn(T val) { return (T(0) < val) - (val < T(0)); }

// // void serialRead(void *params) {
// //   vexGenericSerialEnable(SERIALPORT - 1, 0);
// //   vexGenericSerialBaudrate(SERIALPORT - 1, 115200);
// //   pros::delay(10);
// //   pros::screen::set_pen(COLOR_BLUE);
// //   double distX, distY = 0;
// //   while (true) {
// //     uint8_t buffer[256];
// //     int bufLength = 256;
// //     int32_t nRead = vexGenericSerialReceive(SERIALPORT - 1, buffer, bufLength);
// //     if (nRead >= 0) {
// //       std::stringstream dataStream("");
// //       bool recordOpticalX, recordOpticalY = false;
// //       for (int i = 0; i < nRead; i++) {
// //         char thisDigit = (char)buffer[i];
// //         if (thisDigit == 'D' || thisDigit == 'I' || thisDigit == 'A' ||
// //             thisDigit == 'X' || thisDigit == 'C' || thisDigit == 'Y') {
// //           recordOpticalX = false;
// //           recordOpticalY = false;
// //         }
// //         if (thisDigit == 'C') {
// //           recordOpticalX = false;
// //           dataStream >> distX;
// //           pros::lcd::print(1, "Optical Flow:");
// //           global_distX = distX * 10;
// //           pros::lcd::print(2, "distX: %.2lf", distX * 10);
// //           dataStream.str(std::string());
// //         }
// //         if (thisDigit == 'D') {
// //           recordOpticalY = false;
// //           dataStream >> distY;
// //           global_distY = distY * 10;
// //           pros::lcd::print(3, "distY: %.2lf", distY * 10);
// //           dataStream.str(std::string());
// //         }
// //         if (recordOpticalX)
// //           dataStream << (char)buffer[i];
// //         if (recordOpticalY)
// //           dataStream << (char)buffer[i];
// //         if (thisDigit == 'X')
// //           recordOpticalX = true;
// //         if (thisDigit == 'Y')
// //           recordOpticalY = true;
// //       }
// //     }
// //     pros::Task::delay(25);
// //   }
// // }

// // void optical_flow_distance() {
// //   pros::lcd::print(
// //       4, "current_distance: %.2lf",
// //       sqrt(global_distX * global_distX + global_distY * global_distY));
// // }

// void brake() { // brakes all base motors
//   luA.brake();
//   ruA.brake();
//   luB.brake();
//   ruB.brake();
//   llA.brake();
//   rlA.brake();
//   llB.brake();
//   rlB.brake();
//   pros::delay(1);
// }

// double wrapAngle(double angle) { // forces the angle to be within the -180 <
//                                  // angle < 180 range
//   if (angle > 180.0)
//     while (angle > 180.0)
//       angle -= 360.0;
//   else if (angle < -180.0)
//     while (angle < -180.0)
//       angle += 360.0;
//   return angle;
// }

// double getNormalizedSensorAngle(
//     pros::Rotation &sensor) { // Converts rotational sensor readings into
//                               // degrees and bounds it between -180 to 180
//   double angle =
//       sensor.get_angle() / 100.0; // Convert from centidegrees to degrees
//   return wrapAngle(
//       angle); // forces the angle to be within the -180 < angle < 180 range
// }



// void set_wheel_angle(float target_angle, double kP_set_wheel = 2.0,
//                      double kI_set_wheel = 0.00, double kD_set_wheel = 0.2) {
//   float left_current_angle = getNormalizedSensorAngle(left_rotation_sensor);
//   float right_current_angle = getNormalizedSensorAngle(right_rotation_sensor);

//   float left_error = target_angle - left_current_angle;
//   float right_error = target_angle - right_current_angle;

//   float left_previous_error = 0;
//   float right_previous_error = 0;

//   float left_integral = 0;
//   float right_integral = 0;

//   // Normalize the errors to be within [-180, 180] degrees
//   while (left_error > 180)
//     left_error -= 360;
//   while (left_error < -180)
//     left_error += 360;
//   while (right_error > 180)
//     right_error -= 360;
//   while (right_error < -180)
//     right_error += 360;

//   // std::cout << "Left Current Angle: " << left_current_angle
//   //           << " Target Angle: " << target_angle
//   //           << " Left Error: " << left_error << std::endl;
//   // std::cout << "Right Current Angle: " << right_current_angle
//   //           << " Target Angle: " << target_angle
//   //           << " Right Error: " << right_error << std::endl;

//   // PID control constants

//   int max_attempts = 15; // Set a maximum number of attempts to adjust the angle
//   int attempts = 0;

//   while ((fabs(left_error) > 2 || fabs(right_error) > 2) &&
//          attempts < max_attempts) {
//     left_current_angle = getNormalizedSensorAngle(left_rotation_sensor);
//     right_current_angle = getNormalizedSensorAngle(right_rotation_sensor);

//     left_error = target_angle - left_current_angle;
//     right_error = target_angle - right_current_angle;

//     while (left_error > 180)
//       left_error -= 360;
//     while (left_error < -180)
//       left_error += 360;
//     while (right_error > 180)
//       right_error -= 360;
//     while (right_error < -180)
//       right_error += 360;

//     left_integral += left_error;
//     right_integral += right_error;

//     float left_derivative = left_error - left_previous_error;
//     float right_derivative = right_error - right_previous_error;

//     left_previous_error = left_error;
//     right_previous_error = right_error;

//     float left_motor_speed = kP_set_wheel * left_error +
//                              kI_set_wheel * left_integral +
//                              kD_set_wheel * left_derivative;
//     float right_motor_speed = kP_set_wheel * right_error +
//                               kI_set_wheel * right_integral +
//                               kD_set_wheel * right_derivative;
//     pros::lcd::print(0, "inside setwheel");
//     pros::lcd::print(1, "left_current_angle: %.lf", left_current_angle);
//     pros::lcd::print(2, "right_current_angle: %.lf", right_current_angle);
//     pros::lcd::print(3, "left_error: %.lf", left_error);
//     pros::lcd::print(4, "right_error: %.lf", right_error);
//     luA.move(left_motor_speed);
//     luB.move(left_motor_speed);
//     llA.move(-left_motor_speed);
//     llB.move(-left_motor_speed);

//     ruA.move(right_motor_speed);
//     ruB.move(right_motor_speed);
//     rlA.move(-right_motor_speed);
//     rlB.move(-right_motor_speed);

//     pros::delay(20); // Refresh rate within the set_wheel_angle function
//     attempts++;
//   }

//   // Stop the motors
//   luA.move(0);
//   luB.move(0);
//   llA.move(0);
//   llB.move(0);
//   ruA.move(0);
//   ruB.move(0);
//   rlA.move(0);
//   rlB.move(0);
// }


// void tareBaseMotorEncoderPositions() // tares all base motor encoder positions
// {
//   luA.tare_position();
//   ruA.tare_position();
//   luB.tare_position();
//   ruB.tare_position();
//   llA.tare_position();
//   rlA.tare_position();
//   llB.tare_position();
//   rlB.tare_position();
//   pros::delay(1);
// }

// void clampVoltage(double lu, double ll, double ru, double rl) {
//   // if any of lu, ll, ru or rl are too big, we need to scale them, and we must
//   // scale them all by the same amount so we dont throw off the proportions
//   if (fabs(lu) > MAX_VOLTAGE || fabs(ll) > MAX_VOLTAGE ||
//       fabs(ru) > MAX_VOLTAGE || fabs(rl) > MAX_VOLTAGE) {
//     // figure out which of lu, ll, ru or rl has the largest magnitude
//     double max = fabs(lu);
//     if (max < fabs(ll))
//       max = fabs(ll);
//     if (max < fabs(ru))
//       max = fabs(ru);
//     if (max < fabs(rl))
//       max = fabs(rl);
//     double VoltageScalingFactor =
//         max / MAX_VOLTAGE; // this will definitely be positive, hence it wont
//                            // change the sign of lu, ll, ru or rl.
//     lu = lu / VoltageScalingFactor;
//     ll = ll / VoltageScalingFactor;
//     ru = ru / VoltageScalingFactor;
//     rl = rl / VoltageScalingFactor;
//   }
// }

// double angle(vector3D v1, vector3D v2) {
//   double dot = v1 * v2;
//   double det = v1.x * v2.y - v1.y * v2.x;
//   return -atan2(det, dot); // atan2 automatically considers the sign to deal
//                            // with the trigonometry quadrant
// }

// void pivotWheels(
//     double Target_angle,
//     double allowed_error) // pivot the left and right wheels by a certain amount
// {
//   allowed_error = fabs(allowed_error);
//   double left_angle,
//       right_angle; // numerical angle for each wheel in radians from -pi to pi
//   // power output for angle component of pid
//   double l_angle_pid = 0.0;
//   double r_angle_pid = 0.0;

//   // pivot angle error
//   double l_angle_error = 9999999999;
//   double r_angle_error = 9999999999;

//   int32_t lu; // voltage variables for the four motor pairs of the base
//   int32_t ll;
//   int32_t ru;
//   int32_t rl;

//   PID left_angle_PID(angle_kP, angle_kI, angle_kD);
//   PID right_angle_PID(angle_kP, angle_kI, angle_kD);

//   while (fabs(l_angle_error) > allowed_error ||
//          fabs(r_angle_error) >
//              allowed_error) { // while we havent reached the target pivot angles
//     // get the current pivot angles of the left and right wheels in radians
//     // subtract 90 degrees because the angle zero is defined as the positive x
//     // axis in the spline math but we want the zero angle to be defined as the
//     // wheels pointing to the front of the robot
//     left_angle = (getNormalizedSensorAngle(left_rotation_sensor)) *
//                  TO_RADIANS; // note that the function getNormalizedSensorAngle
//                              // already implements wrapAngle to bound the angle
//                              // between -180 and 180 degrees
//     right_angle =
//         (getNormalizedSensorAngle(right_rotation_sensor)) * TO_RADIANS;
//     // update pivot angle errors

//     vector3D current_left_vector =
//         vector3D(cos(left_angle), sin(left_angle), 0.0);
//     vector3D current_right_vector =
//         vector3D(cos(right_angle), sin(right_angle), 0.0);
//     vector3D target_left_vector =
//         vector3D(cos(Target_angle), sin(Target_angle), 0.0);
//     vector3D target_right_vector =
//         vector3D(cos(Target_angle), sin(Target_angle), 0.0);
//     l_angle_error = angle(current_left_vector, target_left_vector);
//     r_angle_error = angle(current_right_vector, target_right_vector);

//     if (std::isnan(l_angle_error) || std::isnan(r_angle_error)) {
//       l_angle_error = 0.0;
//       r_angle_error = 0.0;
//     }

//     // calculate the PID output
//     l_angle_pid = left_angle_PID.step(l_angle_error);
//     r_angle_pid = right_angle_PID.step(r_angle_error);

//     lu = (int32_t)(l_angle_pid *
//                    scale); // this side seems less powerful on the robot
//     ll = (int32_t)(-l_angle_pid * scale);
//     ru = (int32_t)(r_angle_pid * scale);
//     rl = (int32_t)(-r_angle_pid * scale);

//     // calculate voltages required to run each motor, and scale them into the
//     // acceptable voltage range so they dont exceed max voltage we have to scale
//     // the voltages because if we don't, it can happen that one or more motors
//     // dont move as fast as we expected because we ordered it to move at a
//     // higher voltage than it can physically achieve, and this will throw off
//     // the proportions of velocity of the four motor pairs, and cause the robot
//     // to move in unexpected ways. Scaling means that sometimes the robot moves
//     // slower than expected, but at least it moves correctly otherwise.
//     clampVoltage(lu, ll, ru, rl); // ensure the voltages are within usable range

//     luA.move_voltage(lu);
//     luB.move_voltage(lu);

//     llA.move_voltage(ll);
//     llB.move_voltage(ll);

//     ruA.move_voltage(ru);
//     ruB.move_voltage(ru);

//     rlA.move_voltage(rl);
//     rlB.move_voltage(rl);

//     pros::delay(5);
//   }
//   pros::lcd::print(0, "B");
//   brake();
//   pros::delay(5);
// }

// void Translation_Wheels(
//     double l_distance, double r_distance,
//     double allowed_error) { // rotate the left and right wheels by a certain
//                             // amount WHILE maintaining pivot angle
//   double l_distance_moved = 0.0; // distance that the left and right wheel moved
//   double r_distance_moved = 0.0;

//   double l_distance_error = l_distance; // distance error of the wheels
//   double r_distance_error = r_distance;

//   // pivot angles of the wheel at the start of the motion, we MUST maintain
//   // these pivot angles so the robot moves straight
//   double l_angleMaintain =
//       (getNormalizedSensorAngle(left_rotation_sensor) - 90.0) *
//       TO_RADIANS; // note that the function getNormalizedSensorAngle already
//                   // implements wrapAngle to bound the angle between -180 and
//                   // 180 degrees
//   double r_angleMaintain =
//       (getNormalizedSensorAngle(right_rotation_sensor) - 90.0) *
//       TO_RADIANS; // note that the function getNormalizedSensorAngle already
//                   // implements wrapAngle to bound the angle between -180 and
//                   // 180 degrees

//   double left_angle,
//       right_angle; // numerical angle for each wheel in radians from -pi to pi
//   // steering angle error
//   double l_error = 0.0;
//   double r_error = 0.0;
//   // power output for angle component of pid
//   double l_angle_pid = 0.0;
//   double r_angle_pid = 0.0;
//   // power output for translation component of pid
//   double l_distance_pid = 0.0;
//   double r_distance_pid = 0.0;
//   // scaling down the power depending on how wrong the wheel aiming angle is
//   // limited by base_v = 0.7 in definitions.h
//   double lscale = 0;
//   double rscale = 0;

//   int32_t lu; // voltage variables for the four motor pairs of the base
//   int32_t ll;
//   int32_t ru;
//   int32_t rl;

//   PID left_angle_PID(angle_kP, angle_kI, angle_kD);
//   PID right_angle_PID(angle_kP, angle_kI, angle_kD);
//   PID left_distance_PID(distance_kP, distance_kI, distance_kD);
//   PID right_distance_PID(distance_kP, distance_kI, distance_kD);

//   tareBaseMotorEncoderPositions(); // tare all base motor encoder positions

//   while (fabs(l_distance_error) > allowed_error ||
//          fabs(r_distance_error) >
//              allowed_error) { // while the left and right wheels have not moved
//                               // the target distance
//     // get the distance that the robot has moved since we started this function
//     // we have to divide by four because the velocity of the wheel is the
//     // average of the velocities of the four motors for that wheel we cannot
//     // just take one motor for each gear of each wheel, since each gear is
//     // powered by two motors which could be running at marginally different
//     // speeds by taking readings from all four motors of each wheel, we get more
//     // accurate results
//     l_distance_moved = ((luA.get_position() + luB.get_position() +
//                          llA.get_position() + llB.get_position()) /
//                         4.0) /
//                        ticks_per_mm;
//     r_distance_moved = ((ruA.get_position() + ruB.get_position() +
//                          rlA.get_position() + rlB.get_position()) /
//                         4.0) /
//                        ticks_per_mm;

//     l_distance_error =
//         l_distance - l_distance_moved; // calculate the error distance
//     r_distance_error = r_distance - r_distance_moved;

//     // get the current pivot angles of the left and right wheels in radians
//     // subtract 90 degrees because the angle zero is defined as the positive x
//     // axis in the spline math but we want the zero angle to be defined as the
//     // wheels pointing to the front of the robot
//     left_angle = (getNormalizedSensorAngle(left_rotation_sensor) - 90.0) *
//                  TO_RADIANS; // note that the function getNormalizedSensorAngle
//                              // already implements wrapAngle to bound the angle
//                              // between -180 and 180 degrees
//     right_angle =
//         (getNormalizedSensorAngle(right_rotation_sensor) - 90.0) * TO_RADIANS;

//     // calculate the error angle
//     vector3D l_target_angle =
//         vector3D(cos(l_angleMaintain), sin(l_angleMaintain), 0);
//     vector3D r_target_angle =
//         vector3D(cos(r_angleMaintain), sin(r_angleMaintain), 0);
//     vector3D l_current_angle = vector3D(cos(left_angle), sin(left_angle), 0);
//     vector3D r_current_angle = vector3D(cos(right_angle), sin(right_angle), 0);

//     l_error = angle(l_current_angle, l_target_angle);
//     r_error = angle(r_current_angle, r_target_angle);

//     // calculate the PID output
//     l_angle_pid = left_angle_PID.step(l_error);
//     r_angle_pid = right_angle_PID.step(r_error);

//     l_distance_pid = left_distance_PID.step(l_distance_error);
//     r_distance_pid = right_distance_PID.step(r_distance_error);
//     lu = (int32_t)(scale *
//                    (l_distance_pid +
//                     l_angle_pid)); // this side seems less powerful on the robot
//     ll = (int32_t)(scale * (l_distance_pid - l_angle_pid));
//     ru = (int32_t)(scale * (r_distance_pid + r_angle_pid));
//     rl = (int32_t)(scale * (r_distance_pid - r_angle_pid));

//     luA.move_velocity(lu);
//     luB.move_velocity(lu);

//     llA.move_velocity(ll);
//     llB.move_velocity(ll);

//     ruA.move_velocity(ru);
//     ruB.move_velocity(ru);

//     rlA.move_velocity(rl);
//     rlB.move_velocity(rl);

//     pros::delay(5);
//   }

//   brake();
//   pros::delay(5);
// }

// void base_PID(double targetDistance, double base_kp,
//               double base_ki, double base_kd, double decelerationThreshold = 0) {
//   // Movement variables
//   double powerL = 0;
//   double powerR = 0;
//   double encdleft = 0;
//   double encdright = 0;
//   double errorLeft = 0;
//   double errorRight = 0;
//   double prevErrorLeft = 0;
//   double prevErrorRight = 0;
//   double totalErrorLeft = 0;
//   double totalErrorRight = 0;

//   double l_angleMaintain =
//       (getNormalizedSensorAngle(left_rotation_sensor) - 90.0) *
//       TO_RADIANS; // note that the function getNormalizedSensorAngle already
//                   // implements wrapAngle to bound the angle between -180 and
//                   // 180 degrees
//   double r_angleMaintain =
//       (getNormalizedSensorAngle(right_rotation_sensor) - 90.0) *
//       TO_RADIANS; // note that the function getNormalizedSensorAngle already
//                   // implements wrapAngle to bound the angle between -180 and
//                   // 180 degrees

//   double left_angle, right_angle;
//   double l_error = 0.0;
//   double r_error = 0.0;
//   // power output for angle component of pid
//   double l_angle_pid = 0.0;
//   double r_angle_pid = 0.0;
//   // 1. Perform turning first

//   bool l_move = true;
//   bool r_move = true;
//   int timeout = 0;

//   PID left_angle_PID(angle_kP, angle_kI, angle_kD);
//   PID right_angle_PID(angle_kP, angle_kI, angle_kD);

//   // 2. Now perform distance movement

//   tareBaseMotorEncoderPositions();
//   // double encoder_degrees = targetDistance * degrees_per_mm;
//   // int encoder_ticks = static_cast<int>(round(encoder_degrees));
//   while (l_move || r_move) {
//     // pivotWheels(0,0);
//     // Get encoder values
//     encdleft = ((luA.get_position() + luB.get_position() + llA.get_position() +
//                  llB.get_position()) /
//                 4.0) /
//                ticks_per_mm;
//     ;
//     encdright = ((ruA.get_position() + ruB.get_position() + rlA.get_position() +
//                   rlB.get_position()) /
//                  4.0) /
//                 ticks_per_mm;

//     left_angle = (getNormalizedSensorAngle(left_rotation_sensor) - 90.0 ) *
//                  TO_RADIANS; // note that the function getNormalizedSensorAngle
//                              // already implements wrapAngle to bound the angle
//                              // between -180 and 180 degrees
//     right_angle =
//         (getNormalizedSensorAngle(right_rotation_sensor) - 90.0 ) * TO_RADIANS;

//     vector3D l_target_angle =
//         vector3D(cos(l_angleMaintain), sin(l_angleMaintain), 0);
//     vector3D r_target_angle =
//         vector3D(cos(r_angleMaintain), sin(r_angleMaintain), 0);
//     vector3D l_current_angle = vector3D(cos(left_angle), sin(left_angle), 0);
//     vector3D r_current_angle = vector3D(cos(right_angle), sin(right_angle), 0);

//     l_error = angle(l_current_angle, l_target_angle);
//     r_error = angle(r_current_angle, r_target_angle);

//     // calculate the PID output
//     l_angle_pid = left_angle_PID.step(l_error);
//     r_angle_pid = right_angle_PID.step(r_error);
//     // Calculate distance error

//     errorLeft = fabs(targetDistance) - fabs(encdleft);
//     errorRight = fabs(targetDistance) - fabs(encdright);
//     totalErrorLeft += errorLeft;
//     totalErrorRight += errorRight;

//     // PID for left motors
//     if (fabs(errorLeft) <= base_error) {
//       powerL = 0.0;
//       l_move = false;
//     } else {
//       // Gradually reduce power as you approach the target
//       if (fabs(errorLeft) < decelerationThreshold) {
//         powerL *= 0.5; // Reduce power to half when close to target
//       } else {
//         powerL = base_kp * errorLeft + base_ki * totalErrorLeft +
//                  base_kd * (errorLeft - prevErrorLeft);
//       }
//       powerL = std::clamp(powerL, 80.0, 175.0);
//     }

//     // PID for right motors
//     if (fabs(errorRight) <= base_error) {
//       powerR = 0.0;
//       r_move = false;
//     } else {
//       // Gradually reduce power as you approach the target
//       if (fabs(errorRight) < decelerationThreshold) {
//         powerR *= 0.5; // Reduce power to half when close to target
//       } else {
//         powerR = base_kp * errorRight + base_ki * totalErrorRight +
//                  base_kd * (errorRight - prevErrorRight);
//       }
//       powerR = std::clamp(powerR, 80.0, 175.0);
//     }

//     if (errorRight < 0 && errorLeft < 0)
//       break;

//     // Move the motors
//     if (targetDistance > 0.0) {

//       luA.move_velocity(powerL + l_angle_pid);
//       luB.move_velocity(powerL + l_angle_pid);
//       llA.move_velocity(powerL - l_angle_pid);
//       llB.move_velocity(powerL - l_angle_pid);
//       ruA.move_velocity(powerR + r_angle_pid);
//       ruB.move_velocity(powerR + r_angle_pid);
//       rlA.move_velocity(powerR - r_angle_pid);
//       rlB.move_velocity(powerR - r_angle_pid);
//     } else if (targetDistance < 0.0) {
//       luA.move_velocity(-powerL + l_angle_pid );
//       luB.move_velocity(-powerL + l_angle_pid);
//       llA.move_velocity(-powerL - l_angle_pid);
//       llB.move_velocity(-powerL - l_angle_pid);
//       ruA.move_velocity(-powerR + r_angle_pid);
//       ruB.move_velocity(-powerR + r_angle_pid);
//       rlA.move_velocity(-powerR - r_angle_pid);
//       rlB.move_velocity(-powerR - r_angle_pid);
//     }

//     if (timeout >= 2600)
//       break;

//     pros::lcd::print(0, "ErrorL: %.lf", errorLeft);
//     pros::lcd::print(1, "ErrorR: %.lf", errorRight);

//     // Update previous errors for the next iteration
//     prevErrorLeft = errorLeft;
//     prevErrorRight = errorRight;

//     pros::delay(2); // Delay to reduce CPU load
//     timeout += 2;
//   }
//   brake();
// }

// void initialize() {
//   pros::lcd::initialize();

//   luA.set_brake_mode(MOTOR_BRAKE_COAST);
//   luB.set_brake_mode(MOTOR_BRAKE_COAST);
//   llA.set_brake_mode(MOTOR_BRAKE_COAST);
//   llB.set_brake_mode(MOTOR_BRAKE_COAST);
//   ruA.set_brake_mode(MOTOR_BRAKE_COAST);
//   ruB.set_brake_mode(MOTOR_BRAKE_COAST);
//   rlA.set_brake_mode(MOTOR_BRAKE_COAST);
//   rlB.set_brake_mode(MOTOR_BRAKE_COAST);
//   // liftL.set_brake_mode(MOTOR_BRAKE_HOLD);
//   // liftR.set_brake_mode(MOTOR_BRAKE_HOLD);

//   // while(!left_rotation_sensor.reset());
//   // while(!right_rotation_sensor.reset());

//   left_rotation_sensor.set_data_rate(5);
//   right_rotation_sensor.set_data_rate(5);

//   left_rotation_sensor.set_position(0);
//   right_rotation_sensor.set_position(0);

//   conveyor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
//   roller.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

//   // pros::Task move_base(moveBase);
// //   pros::Task serial_read(serialRead);
// //   pros::Task mehmeh(optical_flow_distance);

//   master.clear();
// }

// void autonomous() { 
    
//     set_wheel_angle(0);
//     base_PID(-300,2.0,0,1.0 ,0); 
//     set_wheel_angle(0);
//     // base_PID(700,5.0,0,0.2 ); 
//     }

// void opcontrol() { autonomous(); }
