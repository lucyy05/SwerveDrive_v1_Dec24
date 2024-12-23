
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


// robot with base(new robot)
#define LEFT_UPPER_BEVEL_MOTOR_1 16   //ROBOT BACK
#define LEFT_UPPER_BEVEL_MOTOR_2 17
#define LEFT_LOWER_BEVEL_MOTOR_1 14   //ROBOT FRONT
#define LEFT_LOWER_BEVEL_MOTOR_2 15
#define RIGHT_UPPER_BEVEL_MOTOR_1 9   //ROBOT BACK
#define RIGHT_UPPER_BEVEL_MOTOR_2 10
#define RIGHT_LOWER_BEVEL_MOTOR_1 4   //ROBOT FRONT
#define RIGHT_LOWER_BEVEL_MOTOR_2 5
#define IMU_PORT 13
#define LEFT_ROTATION_SENSOR_PORT 18
#define RIGHT_ROTATION_SENSOR_PORT 8


// test base
//#define LEFT_UPPER_BEVEL_MOTOR_1 20 //ROBOT BACK
//#define LEFT_UPPER_BEVEL_MOTOR_2 19
//#define LEFT_LOWER_BEVEL_MOTOR_1 11 //ROBOT FRONT
//#define LEFT_LOWER_BEVEL_MOTOR_2 12 
//#define RIGHT_UPPER_BEVEL_MOTOR_1 8 //ROBOT BACK
//#define RIGHT_UPPER_BEVEL_MOTOR_2 9
//#define RIGHT_LOWER_BEVEL_MOTOR_1 2 //ROBOT FRONT
//#define RIGHT_LOWER_BEVEL_MOTOR_2 3
//#define LEFT_ROTATION_SENSOR_PORT 13
//#define RIGHT_ROTATION_SENSOR_PORT 1
//#define IMU_PORT 17


#define POTENTIOMETER_SENSOR_PORT 'H'
#define SOLENOID_SENSOR_PORT 'G'

#define CONVEYOR_THRES_PROX 50
#define CONVEYOR_OPTICAL 1
#define CONVEYOR_MOTOR 7
#define ROLLER_MOTOR 6

#define ZERO_VECTOR INFINITY

#define IMU_PORT 17

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

// pros::Motor intakeLower(UPPER_INTAKE_MOTOR, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
// pros::Motor intakeUpper(LOWER_INTAKE_MOTOR, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);

// pros::Motor liftL(LEFT_LIFT_MOTOR, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
// pros::Motor liftR(RIGHT_LIFT_MOTOR, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);


pros::Rotation left_rotation_sensor(LEFT_ROTATION_SENSOR_PORT, true);
pros::Rotation right_rotation_sensor(RIGHT_ROTATION_SENSOR_PORT, true);
// pros::Imu imu(IMU_SENSOR_PORT);

// CONVEYOR AND ROLLER
pros::Motor conveyor(CONVEYOR_MOTOR, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor roller(ROLLER_MOTOR, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Optical conveyor_optical(CONVEYOR_OPTICAL);

// pros::ADIAnalogIn lifter(POTENTIOMETER_SENSOR_PORT);
// pros::ADIDigitalOut solenoid(SOLENOID_SENSOR_PORT);

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

const double angle_kP_left = 20.0;   //swerve wheel pivoting in driver control and auton
const double angle_kI_left = 0.00;
const double angle_kD_left = 5000.0;

const double angle_kP_right = 20.0;   //swerve wheel pivoting in driver control and auton
const double angle_kI_right = 0.00;
const double angle_kD_right = 5000.0;


const double distance_kP = 0.4; //swerve wheel rotation distance for auton
const double distance_kI = 0.0;
const double distance_kD = 500.0;

const double MAX_VOLTAGE = 12800;

const double velocity_kP = 0.002;   //swerve wheel rotation velocity for driver
const double velocity_kI = 0.000;   //tune for translate
const double velocity_kD = 160;

const double azim_kP = 0.03; //azimuth, for correcting rotation
const double azim_kI = 0.0;    //drunk
const double azim_kD = 10.0;

const double ANGULAR_THRESH = 0.000; // Threshold under which to ignore angular error

const double r_kF = 0.0;   //feedforward compensation for rotation //flick
const double r_kF_STATIC = 0.7; //FF STATIC for rotation
const double v_kF = 0.3;    //feedforward compensation for translation
const double scale  = 25.0;
const double base_v = 0.7; //this defines the min power of the robot when scaling its power down for each side when the wheels are aiming the wrong way

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

//Serial
double global_distY = 0.0;
double global_distX = 0.0;
double global_errorY = 0.0;
double global_errorX = 0.0;

