<h1>This is a code documentation for the main file in the SwerveDrive_v1_Dec24 repository.</h1>

<h2>void setBrakeModes()</h2>

Parameter: No parameters

Return type: No return value

Purpose:

Sets the brake modes for all drive motors and key actuators. Prevents unintended drift and keeps mechanisms in place after movement ends.

E_MOTOR_BRAKE_HOLD – Locks motor position when stopped (for precise holds).

E_MOTOR_BRAKE_BRAKE – Applies resistance to slow down but allows movement when force is applied

<h2>void setMotorCurrentLimit(int current)</h2>

Parameter: current

Parameter Range: 0 to 2500 mA

Return type: No return value

Purpose: Set current limit for all motors

<h2>serialRead(void* params)</h2>

Parameter: void* params – Generic pointer (not directly used, can pass NULL)

Return type: No return value

Purpose:

- Reading data from a serial port.

- Parsing the incoming data stream to extract positional information (dist_X, dist_Y).

- Calculating velocities based on the changes in positional data.

<h2>void brake()</h2>

Parameter: No parameters

Purpose: Apply brakes to all base motors

<h2>void tareBaseMotorEncoderPositions()</h2>

Parameter: No parameters

Return type: No return value

Purpose:

Ensure that all motor encoders are synchronized to zero. 

This prevents drift in encoder readings and maintains the accuracy of distance and positional calculations.

<h2>void clampVoltage(int32_t VOLTAGE)</h2>

Parameter: Voltage

Parameter Range: 0 to 12V

Return type: No return value

Purpose:
Ensures that the motor voltage values (lu, ll, ru, rl) do not exceed the specified VOLTAGE threshold

<h2>void move_voltage_wheels(int32_t lu, int32_t ll, int32_t ru, int32_t rl)</h2>

Parameter: voltage for left upper | left lower | right upper | right lower motors

Parameter Range: -12V to 12V

Return type: No return value

Purpose: Controls voltage sent to motors

<h2>double wrapAngle(double angle)</h2>

Parameter: angle (in degrees)

Parameter Range: -inf to +inf

Return type: double, angle wrapped in -180 < angle <= 180

Purpose: Forces angle to stay within -180 to 180 degrees

<h2>getNormalizedSensorAngle(pros::Rotation &sensor)</h2>

Parameter: pros::Rotation &sensor, aka reference to a rotation sensor object, measures angular position

Return type: Normalised angle (degrees)

Return range: -180 < angle <= 180

Purpose: Reads rotation sensor value (in centidegrees) and converts it to degrees by division by 100

<h2>vector3D normalizeJoystick(int x_in, int y_in)</h2>

Parameters:
int x_in: Joystick input along x-axis
Range: -127 to 127

int y_in: Joystick input along y-axis
Range: -127 to 127

Return type: vector3D: 2D vector representing the joystick direction and magnitude
X and Y components scaled between -1.0 to 1.0

Purpose:
- Converts the joystick input (X, Y) to a polar vector
- Corrects for joystick deadband to ignore small unintended movements (stick drift)

<h2>vector3D normalizeRotation(int x_in)</h2>

Parameters:
int x_in: Joystick input for rotation
Range: -127 to 127

Return Type:
vector3D – Rotational vector. Z-component scaled between -1.0 to 1.0.

Purpose:
- Converts the joystick’s X-axis rotation input to a scaled rotational vector
- Applies a deadband to filter out small joystick movements
- Positive X-input results in negative rotation (clockwise), while negative X-input results in positive rotation (counterclockwise)

<h2>double angle(vector3D v1, vector3D v2)</h2>

Parameters:

vector3D v1: First vector, represents direction/position in 2D/3D space

vector3D v2: Second vector, represents another direction/position

<br>Return type: double -- Angle between 2 vectors (radians)

Return range: -pi to pi

<br>Purpose:

Computes the signed angle between 2 vectors using dot product and determinant.



<br>Dot product: How much 2 vectors "align"

+ve value: vectors point in the same directions

0: vectors are perpendicular

-ve value: vectors point in opposite directions



<br>Determinant: How much 1 vector "rotates" from another

+ve value: v2 rotated CCW from v1

0: vectors are collinear

-ve value: v2 rotated CW from v1

atan(y, x)
- computes signed angle (aka shortest rotational path) betw 2 vectors

<h2>double max(double a, double b)</h2>

Parameters: double a, double b

Return type: the larger of the inputs, data type double 

<h2>double min(double a, double b)</h2>

Parameters: double a, double b

Return type: the smaller of the inputs, data type double

<h2>void moveBase()</h2>

Parameter: No parameters

Return type: No return value

Purpose:
moveBase(): driver control code

- It processes joystick inputs to compute target velocities for translation and rotation.

- Uses PID controllers to adjust wheel angles and velocities, ensuring smooth and precise robot movement.

- Calculates angular error to ensure the robot adjusts its orientation during rotation.

- Dynamically scales motor power based on battery voltage to maintain consistent performance.

<h2>bool isMotorAtTarget(int port, int target)</h2>

Parameters: 
int port, aka Motor port number

Parameter Ranges:
Port: 1 to 21
target: target motor position in encoder ticks

Return type:
bool - true if motor is within +/- 5 ticks of the target, otherwise false

<h2>void slamDunk()</h2>

Parameter: No parameters

Return type: No return value

Purpose:
- controls the robot arm mechanism for scoring wall stakes

- uses a PID-like method to calculate motor power and move the arm to different positions based on the slammingState

NOTE<br>
SLAM_START_STATE – Resting position.

SLAM_MID_STATE – Midpoint or holding position.

SLAM_EXTENDED_STATE – Fully extended.

<h2>void moveBaseAutonomous(double targetX, double targetY, double target_heading)</h2>

Parameters:

double targetX – Target distance along the X-axis (millimeters)

double targetY – Target distance along the Y-axis (millimeters)

double target_heading – Target rotational heading (degrees)

Return type: No return value

Purpose:
- Controls the robot’s autonomous movement to reach a specific position and orientation by calculating wheel velocities and angles.

- Utilizes PID controllers to adjust the robot’s translational and rotational movement, ensuring smooth and precise navigation.

- Continuously checks for positional errors and adjusts wheel voltages to correct the path until the robot reaches the target.

<h2>void turn45() | void turn90() | void turn180() </h2>

Parameter: No parameters

Return type: No return value

Purpose:
Rotates the robot by 45 | 90 | 180 degrees.

