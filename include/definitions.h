//zihui's branch for zihui's changes
#include <cstdint>
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "pros/rotation.hpp"
#include <cmath>
#include <sstream>
#include <vector>
//#include "api.h"
#include "vector.h"
#include "pros/imu.hpp"

// #define LEFT_UPPER_BEVEL_MOTOR_1 15
// #define LEFT_UPPER_BEVEL_MOTOR_2 12 
// #define LEFT_LOWER_BEVEL_MOTOR_1 1
// #define LEFT_LOWER_BEVEL_MOTOR_2 2
// #define RIGHT_UPPER_BEVEL_MOTOR_1 20
// #define RIGHT_UPPER_BEVEL_MOTOR_2 19
// #define RIGHT_LOWER_BEVEL_MOTOR_1 9
// #define RIGHT_LOWER_BEVEL_MOTOR_2 7

// #define LEFT_UPPER_BEVEL_MOTOR_1 11
// #define LEFT_UPPER_BEVEL_MOTOR_2 12
// #define LEFT_LOWER_BEVEL_MOTOR_1 19
// #define LEFT_LOWER_BEVEL_MOTOR_2 20
// #define RIGHT_UPPER_BEVEL_MOTOR_1 1
// #define RIGHT_UPPER_BEVEL_MOTOR_2 2
// #define RIGHT_LOWER_BEVEL_MOTOR_1 9
// #define RIGHT_LOWER_BEVEL_MOTOR_2 10

// #define LEFT_LIFT_MOTOR 18
// #define RIGHT_LIFT_MOTOR 8

// #define UPPER_INTAKE_MOTOR 15
// #define LOWER_INTAKE_MOTOR 14

// #define IMU_SENSOR_PORT 16
// #define SERIALPORT 17

// #define LEFT_ROTATION_SENSOR_PORT 13
// #define RIGHT_ROTATION_SENSOR_PORT 3

// #define LEFT_LIFT_MOTOR 18
// #define RIGHT_LIFT_MOTOR 10

// #define UPPER_INTAKE_MOTOR 15
// #define LOWER_INTAKE_MOTOR 14


// #define SERIALPORT 16

// robot with base(new robot)
#define LEFT_UPPER_BEVEL_MOTOR_1 16
#define LEFT_UPPER_BEVEL_MOTOR_2 17
#define LEFT_LOWER_BEVEL_MOTOR_1 14
#define LEFT_LOWER_BEVEL_MOTOR_2 15
#define RIGHT_UPPER_BEVEL_MOTOR_1 9
#define RIGHT_UPPER_BEVEL_MOTOR_2 10
#define RIGHT_LOWER_BEVEL_MOTOR_1 4
#define RIGHT_LOWER_BEVEL_MOTOR_2 5


// #define LEFT_UPPER_BEVEL_MOTOR_1 14
// #define LEFT_UPPER_BEVEL_MOTOR_2 15
// #define LEFT_LOWER_BEVEL_MOTOR_1 16
// #define LEFT_LOWER_BEVEL_MOTOR_2 17
// #define RIGHT_UPPER_BEVEL_MOTOR_1 4
// #define RIGHT_UPPER_BEVEL_MOTOR_2 5
// #define RIGHT_LOWER_BEVEL_MOTOR_1 9
// #define RIGHT_LOWER_BEVEL_MOTOR_2 10


#define LEFT_ROTATION_SENSOR_PORT 18
#define RIGHT_ROTATION_SENSOR_PORT 8

// robot without base
// #define LEFT_UPPER_BEVEL_MOTOR_1 11
// #define LEFT_UPPER_BEVEL_MOTOR_2 12
// #define LEFT_LOWER_BEVEL_MOTOR_1 19
// #define LEFT_LOWER_BEVEL_MOTOR_2 20
// #define RIGHT_UPPER_BEVEL_MOTOR_1 2
// #define RIGHT_UPPER_BEVEL_MOTOR_2 3
// #define RIGHT_LOWER_BEVEL_MOTOR_1 8
// #define RIGHT_LOWER_BEVEL_MOTOR_2 9

// #define LEFT_ROTATION_SENSOR_PORT 13
// #define RIGHT_ROTATION_SENSOR_PORT 1



#define SOLENOID_SENSOR_PORT 'G'
#define front_roller_adi 'H'
#define mobilegoal_bottom 'D'

#define SLAM_DUNK_SENSOR_PORT 'A'
#define SLAM_DUNK_SOLENOID 'B'

#define CONVEYOR_MOTOR 7
#define ROLLER_MOTOR 6

#define SERIALPORT 20
#define IMU_SENSOR_PORT 2
// #define IMU_SENSOR_PORT 13
#define SLAM_DUNK_MOTOR 3

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

pros::IMU imu(IMU_SENSOR_PORT);

// pros::Motor intakeLower(UPPER_INTAKE_MOTOR, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
// pros::Motor intakeUpper(LOWER_INTAKE_MOTOR, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);

// pros::Motor liftL(LEFT_LIFT_MOTOR, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
// pros::Motor liftR(RIGHT_LIFT_MOTOR, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);


pros::Rotation left_rotation_sensor(LEFT_ROTATION_SENSOR_PORT, true);
pros::Rotation right_rotation_sensor(RIGHT_ROTATION_SENSOR_PORT, true);
pros::Motor slam_dunkkkk(SLAM_DUNK_MOTOR, pros::E_MOTOR_GEAR_RED, false, pros::E_MOTOR_ENCODER_DEGREES);

// CONVEYOR AND ROLLER
pros::Motor conveyor(CONVEYOR_MOTOR, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor roller(ROLLER_MOTOR, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

// pros::ADIAnalogIn lifter(POTENTIOMETER_SENSOR_PORT);
pros::ADIDigitalOut solenoid(SOLENOID_SENSOR_PORT);
pros::ADIDigitalOut front_roller(front_roller_adi);
pros::ADIDigitalOut slam_in_out(SLAM_DUNK_SOLENOID);
pros::ADIDigitalOut mobilegoal_bot(mobilegoal_bottom);

pros::ADIAnalogIn slam_dunk(SLAM_DUNK_SENSOR_PORT);

extern "C" int32_t vexGenericSerialReceive( uint32_t index, uint8_t *buffer, int32_t length );
extern "C" void vexGenericSerialEnable(  uint32_t index, uint32_t nu );
extern "C" void vexGenericSerialBaudrate(  uint32_t index, uint32_t rate );
extern "C" int32_t vexGenericSerialTransmit( uint32_t index, uint8_t *buffer, int32_t length );

//Controllers
int leftX = 0, leftY = 0, rightX = 0;

//PARAMETERS
const double DEADBAND =  8.0;
const double MAX_RPM = 600.0;
const double TRANSLATE_RATIO = 1.0;
const double ROTATE_RATIO = 3.0;
const double WHEEL_RADIUS = 34.925;
const double WHEEL_BASE_RADIUS = 161.50;   // mm
const double MAX_SPEED = (2.0*M_PI*WHEEL_RADIUS*MAX_RPM)/60.0 ; //mm per second
const double SPEED_TO_RPM = 60.0/(2.0*M_PI*WHEEL_RADIUS);
const double MAX_ANGULAR = MAX_SPEED/WHEEL_BASE_RADIUS; // rad/s
const double ACCEL = 8000.0;    // mm/s2
const double ANGULAR_ACCEL = 1.0;   // rad/s2
const double SCALING_FACTOR = MAX_RPM / 127.0;
const double TO_DEGREES = (180.0 / M_PI);
const double TO_RADIANS = (M_PI / 180.0);
const double THETA_MAX = 15.0;



//moving (moveBase)
vector3D target_v;
vector3D target_r;
vector3D temp;
vector3D v_right;
vector3D v_left;
double theta; // angle between direction vector and robot right, radians

const double angle_kP = 28.0;   //swerve wheel pivoting in driver control and auton
const double angle_kI = 0.00;
const double angle_kD = 5000.0;

const double angle_kP_mehmeh = 10.0;   //swerve wheel pivoting in driver control and auton
// const double angle_kI = 0.00;
// const double angle_kD = 5000.0;

bool toggleBase = false;

const double angle_kP_left = 28.0;   //swerve wheel pivoting in driver control and auton
const double angle_kI_left = 0.0;
const double angle_kD_left = 1200.0; // was 2000

const double angle_kP_right = 26.0;   //swerve wheel pivoting in driver control and auton
const double angle_kI_right = 0.0001;
const double angle_kD_right = 1200.0; // was 2000



// const double distance_kP = 0.4; //swerve wheel rotation distance for auton
// const double distance_kI = 0.0;
// const double distance_kD = 500.0;

const double distance_kP = 0.4; //swerve wheel rotation distance for auton
const double distance_kI = 0.0;
const double distance_kD = 500.0;
const double MAX_VOLTAGE = 12800;

const double velocity_kP = 0.004;   //swerve wheel rotation velocity for driver
const double velocity_kI = 0.000;
const double velocity_kD = 90;

const double azim_kP = 15.0; //azimuth, for correcting rotation
const double azim_kI = 0.0;    //drunk
const double azim_kD = 1.0;


const double r_kF = 0.12;   //feedforward compensation for rotation //flick
const double v_kF = 0.32;    //feedforward compensation for translation

const double scale  = 40.0;
const double base_v = 0.7; //this defines the min power of the robot when scaling its power down for each side when the wheels are aiming the wrong way

// const double ticks_per_mm = 2.5; //convert mm to ticks
const double ticks_per_mm = 1.7;
double base_error = 2.0;
// double decelerationThreshold = 0;


double target_angle = 0.0;
double target_angleL = 0.0;
double target_angleR = 0.0;
double left_turn_speed = 0.0;
double right_turn_speed = 0.0;
double rotational = 0.0;
double rotationalL = 0.0;
double rotationalR = 0.0;



int liftTarget = 0;
bool liftEnable = false;

bool isLeftFlipped = false;
bool isRightFlipped = false;

//Serial
double global_distY = 0.0;
double global_distX = 0.0;
double global_errorY = 0.0;
double global_errorX = 0.0;
double original_x = 0;

bool mobile_goal_actuated = false;
bool mobile_goal_jaw = false;

bool front_roller_actuated = false;

bool slam_dunk_actuated = false;

int slammingState = 0;
double slam_target = 0;
double slam_Kp = 0.4;
double slam_Kd = 0.1;
double slam_Ki = 0.0;

bool is_turning = false;
double turn_target_angle;
double turn_previous_error = 0;

double mehmeh;
double mehmeh2;

const double ANGULAR_THRESH = 0.000;
const double r_kF_STATIC = 0.7;
double imu_angle;

const double ALPHA = 0.85;
const double BETA = 0.38;
const double THRESHOLD = 250.0;
const double height_from_gnd = 20.0;    //Height in mm
const double scaler = 7.2;              //Adjust for sensitivity for different surfaces
const double scale_factor = height_from_gnd * 2.0 * tan(42.0 / 2.0) / (35.0 * scaler);