#include "definitions.h"
#include "api.h"

// void turn90(){
//     auton_target_x = 0.0;
//     auton_target_y = 0.0;
//     //without mogo
//     // auton_heading_kP = 0.09;
//     // auton_heading_kI = 0.0;
//     // auton_heading_kD = 0.05;

//     //with empty mogo
//     auton_heading_kP = 0.09;
//     auton_heading_kI = 0.0;
//     auton_heading_kD = 0.12;
//     auton_target_heading = 90.0;
// }

// void turn45(){
//     auton_target_x = 0.0;
//     auton_target_y = 0.0;
//     //without mogo
//     // auton_heading_kP = 0.12;
//     // auton_heading_kI = 0.0;
//     // auton_heading_kD = 0.03;

//     //with empty mogo
//     auton_heading_kP = 0.12;
//     auton_heading_kI = 0.0;
//     auton_heading_kD = 0.2;
//     auton_target_heading = 45.0;
// }

// void turn180(){
//     auton_target_x = 0.0;
//     auton_target_y = 0.0;
//     //without mogo
//     // auton_heading_kP = 0.058;
//     // auton_heading_kI = 0.0;
//     // auton_heading_kD = 0.031;

//     //with full mogo
//     auton_heading_kP = 0.0625;
//     auton_heading_kI = 0.00;
//     auton_heading_kD = 0.31;
//     auton_target_heading = 180.0;
// }

void translate(double target_x, double target_y, double target_heading){
    auton_target_heading = target_heading;
    auton_target_x = target_x;
    auton_target_y = target_y;
}

// void turn90(){
//     // auton_heading_kP = 0.09;
//     // auton_heading_kI = 0.0;
//     // auton_heading_kD = 0.05;
//     auton_heading_kP = 0.09;
//     auton_heading_kI = 0.0;
//     auton_heading_kD = 0.12;
//     moveBaseAutonomous(0.0, 0.0, 90.0);
// }

// void turn45(){
//     // auton_heading_kP = 0.12;
//     // auton_heading_kI = 0.0;
//     // auton_heading_kD = 0.03;
//     auton_heading_kP = 0.12;
//     auton_heading_kI = 0.0;
//     auton_heading_kD = 0.2;
//     moveBaseAutonomous(0.0, 0.0, 45.0);
// }

// void turn180(){
//     // auton_heading_kP = 0.058; //Without Mogo
//     // auton_heading_kI = 0.0;
//     // auton_heading_kD = 0.031;
//     auton_heading_kP = 0.0625; //Full stack
//     auton_heading_kI = 0.0001;
//     auton_heading_kD = 0.31;
//     moveBaseAutonomous(0.0, 0.0, 180.0);
// }