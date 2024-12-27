#pragma once
#include <cstdint>
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "pros/rotation.hpp"
#include <cmath>
#include <sstream>
#include <vector>
#include "vector.h"
#include "pros/imu.hpp"
//CAVEN CODE COPY, FOR IPIN

//#include "api.h"

/* robot with base (UPIN) */
// #define LEFT_UPPER_BEVEL_MOTOR_1 16   //ROBOT BACK
// #define LEFT_UPPER_BEVEL_MOTOR_2 17
// #define LEFT_LOWER_BEVEL_MOTOR_1 14   //ROBOT FRONT
// #define LEFT_LOWER_BEVEL_MOTOR_2 15
// #define RIGHT_UPPER_BEVEL_MOTOR_1 9   //ROBOT BACK
// #define RIGHT_UPPER_BEVEL_MOTOR_2 10
// #define RIGHT_LOWER_BEVEL_MOTOR_1 4   //ROBOT FRONT
// #define RIGHT_LOWER_BEVEL_MOTOR_2 5
// #define IMU_PORT_1 2
// #define LEFT_ROTATION_SENSOR_PORT 18
// #define RIGHT_ROTATION_SENSOR_PORT 8

/* robot with base (IPIN) */
#define LEFT_UPPER_BEVEL_MOTOR_1 16   //ROBOT BACK
#define LEFT_UPPER_BEVEL_MOTOR_2 17
#define LEFT_LOWER_BEVEL_MOTOR_1 14   //ROBOT FRONT
#define LEFT_LOWER_BEVEL_MOTOR_2 15
#define RIGHT_UPPER_BEVEL_MOTOR_1 9   //ROBOT BACK
#define RIGHT_UPPER_BEVEL_MOTOR_2 10
#define RIGHT_LOWER_BEVEL_MOTOR_1 4   //ROBOT FRONT
#define RIGHT_LOWER_BEVEL_MOTOR_2 5
// #define IMU_PORT_1 13
// #define IMU_PORT_2 12
#define IMU_PORT 13
#define LEFT_ROTATION_SENSOR_PORT 18
#define RIGHT_ROTATION_SENSOR_PORT 8

/* test base (NO PAYLOAD)*/
// #define LEFT_UPPER_BEVEL_MOTOR_1 16   //ROBOT BACK
// #define LEFT_UPPER_BEVEL_MOTOR_2 17
// #define LEFT_LOWER_BEVEL_MOTOR_1 14   //ROBOT FRONT
// #define LEFT_LOWER_BEVEL_MOTOR_2 15
// #define RIGHT_UPPER_BEVEL_MOTOR_1 9   //ROBOT BACK
// #define RIGHT_UPPER_BEVEL_MOTOR_2 10
// #define RIGHT_LOWER_BEVEL_MOTOR_1 4   //ROBOT FRONT
// #define RIGHT_LOWER_BEVEL_MOTOR_2 5
// #define IMU_PORT 13
// #define LEFT_ROTATION_SENSOR_PORT 18
// #define RIGHT_ROTATION_SENSOR_PORT 8

#define SLAM_DUNK_SENSOR_PORT 'A'
#define SLAM_DUNK_SOLENOID 'B'

#define SOLENOID_SENSOR_PORT 'D'
#define mobilegoal_bottom 'G'
#define POTENTIOMETER_SENSOR_PORT 'H'
#define YOINKE_SENSOR_PORT 'F'

#define CONVEYOR_OPTICAL 1
#define CONVEYOR_THRES_PROX 130

#define SLAM_DUNK_MOTOR 3

#define ROLLER_MOTOR 6
#define CONVEYOR_MOTOR 7

#define SERIALPORT 20

#define ZERO_VECTOR INFINITY

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Motor luA(LEFT_UPPER_BEVEL_MOTOR_1, pros::E_MOTOR_GEARSET_06,  false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor luB(LEFT_UPPER_BEVEL_MOTOR_2, pros::E_MOTOR_GEARSET_06,  false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor llA(LEFT_LOWER_BEVEL_MOTOR_1, pros::E_MOTOR_GEARSET_06,  false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor llB(LEFT_LOWER_BEVEL_MOTOR_2, pros::E_MOTOR_GEARSET_06,  false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor ruA(RIGHT_UPPER_BEVEL_MOTOR_1, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor ruB(RIGHT_UPPER_BEVEL_MOTOR_2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rlA(RIGHT_LOWER_BEVEL_MOTOR_1, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rlB(RIGHT_LOWER_BEVEL_MOTOR_2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::IMU imu(IMU_PORT);
// pros::IMU imu2(IMU_PORT_2);

pros::Motor slam_dunk_motor(SLAM_DUNK_MOTOR, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::ADIDigitalOut slam_in_out(SLAM_DUNK_SOLENOID);
pros::ADIAnalogIn slam_dunk(SLAM_DUNK_SENSOR_PORT);
pros::ADIDigitalOut solenoid(SOLENOID_SENSOR_PORT);
pros::ADIDigitalOut mobilegoal_bot(mobilegoal_bottom);
pros::ADIDigitalOut yoinker(YOINKE_SENSOR_PORT);

// pros::Motor intakeLower(UPPER_INTAKE_MOTOR, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
// pros::Motor intakeUpper(LOWER_INTAKE_MOTOR, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);

// pros::Motor liftL(LEFT_LIFT_MOTOR, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
// pros::Motor liftR(RIGHT_LIFT_MOTOR, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::Rotation left_rotation_sensor(LEFT_ROTATION_SENSOR_PORT, true);
pros::Rotation right_rotation_sensor(RIGHT_ROTATION_SENSOR_PORT, true);

// CONVEYOR AND ROLLER
pros::Motor conveyor(CONVEYOR_MOTOR, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor roller(ROLLER_MOTOR, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Optical conveyor_optical(CONVEYOR_OPTICAL);

// ROLLER LIFT
pros::ADIDigitalOut roller_lifter(POTENTIOMETER_SENSOR_PORT);
// pros::ADIDigitalOut solenoid(SOLENOID_SENSOR_PORT);

extern "C" int32_t vexGenericSerialReceive( uint32_t index, uint8_t *buffer, int32_t length );
extern "C" void vexGenericSerialEnable(  uint32_t index, uint32_t nu );
extern "C" void vexGenericSerialBaudrate(  uint32_t index, uint32_t rate );
extern "C" int32_t vexGenericSerialTransmit( uint32_t index, uint8_t *buffer, int32_t length );


/* Controllers */
int leftX = 0, leftY = 0, rightX = 0, rightY = 0;

/* Parameters START */
const double DEADBAND = 8.0;
const double MAX_RPM = 600.0;
const double TRANSLATE_RATIO = 1.0;
const double ROTATE_RATIO = 3.0;
const double WHEEL_RADIUS = 34.925;
const double WHEEL_BASE_RADIUS = 161.50;    // mm
const double MAX_SPEED = (2.0*M_PI*WHEEL_RADIUS*MAX_RPM)/60.0;  //mm per second
const double SPEED_TO_RPM = 60.0/(2.0*M_PI*WHEEL_RADIUS);
const double MAX_ANGULAR = MAX_SPEED/WHEEL_BASE_RADIUS; // rad/s
const double MAX_ANGULAR_SCALE = 0.3;
const double TO_DEGREES = (180.0 / M_PI);
const double TO_RADIANS = (M_PI / 180.0);
const double MAX_VOLTAGE = 12000.0;
const int MAX_CURRENT_BASE = 1800;
const int VOLTAGE_CUTOFF = 2700; // mV

//moving (moveBase)
vector3D target_v;
vector3D target_r;
vector3D temp;
vector3D v_right;
vector3D v_left;

//voltages
int32_t lu; // left upper 
int32_t ll; // left lower 
int32_t ru; // right upper 
int32_t rl; // right lower 

/* Driver constants START */
// Swerve wheel pivoting
const double angle_kP_left = 30.0;
const double angle_kI_left = 0.0;
const double angle_kD_left = 7000.0;

const double angle_kP_right = 30.0;
const double angle_kI_right = 0.0;
const double angle_kD_right = 7000.0;

const double velocity_kP = 0.0020;   //swerve wheel rotation velocity for driver
const double velocity_kI = 0.0;     //tune for translate
const double velocity_kD = 150.0;

const double distance_kP = 50.0; //swerve wheel rotation distance
const double distance_kI = 0.0;
const double distance_kD = 500.0;

const double azim_kP = 0.03; //azimuth, for correcting rotation //.16
const double azim_kI = 0.0;    //drunk
const double azim_kD = 10.0;    //168000

const double ANGULAR_THRESH = 0.0; // Threshold under which to ignore angular error

const double r_kF = 0.0;   //feedforward compensation for rotation //flick
const double r_kF_STATIC = 0.7; //FF STATIC for rotation
const double v_kF = 0.4;    //feedforward compensation for translation
const double scale = 25.0;
const double base_v = 0.7; //this defines the min power of the robot when scaling its power down for each side when the wheels are aiming the wrong way
/* Driver constants END */

/* Autonomous constants START */
// Swerve wheel pivoting
const double auton_angle_kP_left = 55.0;
const double auton_angle_kI_left = 0.0;
const double auton_angle_kD_left = 5000.0;

const double auton_angle_kP_right = 55.0;
const double auton_angle_kI_right = 0.0;
const double auton_angle_kD_right = 5000.0;

const double auton_l_velocity_kP = 0.0005;   //swerve wheel rotation velocity for auton
const double auton_l_velocity_kI = 0.000;     //tune for translate
const double auton_l_velocity_kD = 200.0;

const double auton_r_velocity_kP = 0.00055;   //swerve wheel rotation velocity for auton
const double auton_r_velocity_kI = 0.000;     //tune for translate
const double auton_r_velocity_kD = 200.0;

double auton_distance_kP = 0.15; //swerve wheel rotation distance
double auton_distance_kI = 0.0;
double auton_distance_kD = 0.2;

// const double auton_distance_kP = 0.15; //swerve wheel rotation distance auton translate left or right
// const double auton_distance_kI = 0.0;
// const double auton_distance_kD = 500.0;

double auton_heading_kP = 0.09; //swerve heading
double auton_heading_kI = 0.0;
double auton_heading_kD = 0.05;

double auton_target_x = 0.0;
double auton_target_y = 0.0;
double auton_target_heading = 0.0;

double auton_azim_kP = 0.03; //azimuth, for correcting rotation
double auton_azim_kI = 0.0;    //drunk
double auton_azim_kD = 10.0;

double AUTON_ANGULAR_THRESH = 0.0; // Threshold under which to ignore angular error
/* Autonomous constants END */

const double ticks_per_mm = 2.5; //convert mm to ticks

double target_angle = 0.0;
double target_angleL = 0.0;
double target_angleR = 0.0;
double left_turn_speed = 0.0;
double right_turn_speed = 0.0;
double rotational = 0.0;
double rotationalL = 0.0;
double rotationalR = 0.0;

//MogoLift
const double mkP = 0.88;
const double mkI = 0.0; 
const double mkD = 1.79;

int liftTarget = 0;
bool liftEnable = false;

bool isLeftFlipped = false;
bool isRightFlipped = false;

//Slam dunk
// int defaultSlamValue = 0;
// Slam dunk constants -- IPIN
const int defaultSlamValue = 2974;

// Slam dunk constants -- UPIN
//const int defaultSlamValue = 2998;

enum SlammingState {
    SLAM_START_STATE = 0,
    SLAM_MID_STATE = 1,
    SLAM_EXTENDED_STATE = 2
};

SlammingState slammingState = SLAM_START_STATE;

bool slam_dunk_actuated = false;

double slam_target = 0.0;
double slam_Kp = 0.45;
double slam_Kd = 0.1;
double slam_Ki = 0.0;

//Serial read
bool serial_task_enabled = false;
double global_distY = 0.0;
double global_distX = 0.0;
double global_errorY = 0.0;
double global_errorX = 0.0;
double optical_v_x = 0.0;
double optical_v_y = 0.0;

//CONVEYOR
int detected_ring_time = 0;

//Mobile goal grabber
bool mobile_goal_actuated = false;
bool mobile_goal_jaw = false;

//Yoinker
bool yoinker_actuated = false;

//Optical flow
const double ALPHA = 0.85;
const double BETA = 0.38;
const double THRESHOLD = 200.0;
const double height_from_gnd = 20.0;    //Height in mm
const double scaler = 7.2;              //Adjust for sensitivity for different surfaces
const double scale_factor = height_from_gnd * 2.0 * tan(42.0 / 2.0) / (35.0 * scaler);

//Roller lift
bool roller_lifts = false;

//Driver
bool driver = false;
bool arcade = false;