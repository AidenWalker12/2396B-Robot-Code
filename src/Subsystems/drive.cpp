#include "subsystems/drive.hpp"

// Motor groups
pros::MotorGroup leftMotors({-1, -2}, pros::MotorGearset::blue);      // Left motors
pros::MotorGroup rightMotors({11, 12}, pros::MotorGearset::blue);     // Right motors

// IMU & encoders
pros::Imu imu(16);                         // Inertial sensor
pros::Rotation horizontalEnc(17);          // Horizontal encoder
pros::Rotation verticalEnc(18);            // Vertical encoder

// Tracking wheels for odometry
lemlib::TrackingWheel horizontal(
    &horizontalEnc,                     // Pointer to horizontal encoder
    lemlib::Omniwheel::NEW_2,     // Wheel type
    -6.7                               // Offset from robot center (inches or cm)
);
lemlib::TrackingWheel vertical(
    &verticalEnc,                       // Pointer to vertical encoder
    lemlib::Omniwheel::NEW_2,     // Wheel type
    -0.5                               // Offset from robot center
);

// Drivetrain and chassis configuration
lemlib::Drivetrain drivetrain(
    &leftMotors,                     // Pointer to left motor group
    &rightMotors,                   // Pointer to right motor group
    10.896752,                       // Track width (distance between wheels)
    lemlib::Omniwheel::NEW_275,   // Wheel type
    600,                                    // Wheelbase (distance between axles)
    6                           // Gear ratio
);

// Exponential drive curves for input shaping
lemlib::ExpoDriveCurve throttleCurve(
    3,                      // Exponent for throttle curve
    10,                    // Deadband
    1.019                      // Multiplier
);
lemlib::ExpoDriveCurve steerCurve(
    3,                      // Exponent for steering curve
    10,                    // Deadband
    1.019                      // Multiplier
);

// PID controller settings for linear movement
lemlib::ControllerSettings linearController(
    10,                           // kP
    0,                            // kI
    3,                            // kD
    0,                   // kF
    0,                    // kS
    0,             // kV
    0,                    // kA
    0,             // kG
    0                           // kStatic
);
// PID controller settings for angular movement
lemlib::ControllerSettings angularController(
    2,                            // kP
    0,                            // kI
    10,                           // kD
    0,                   // kF
    0,                    // kS
    0,             // kV
    0,                    // kA
    0,             // kG
    0                           // kStatic
);

// Odometry sensor configuration
lemlib::OdomSensors sensors(
    &vertical,                    // Vertical tracking wheel
    nullptr,                      // No second vertical tracking wheel
    &horizontal,                  // Horizontal tracking wheel
    nullptr,                      // No second horizontal tracking wheel
    &imu                          // Inertial Measurement Unit
);

// Main chassis object for robot control
lemlib::Chassis chassis(
    drivetrain,                   // Drivetrain configuration
    linearController,             // Linear PID controller
    angularController,            // Angular PID controller
    sensors,                      // Odometry sensors
    &throttleCurve,               // Throttle input curve
    &steerCurve                   // Steering input curve
);

void initDrive() {
    chassis.calibrate();
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}
