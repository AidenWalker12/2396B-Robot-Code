#include "lemlib/chassis/trackingWheel.hpp"
#include "main.h"


//* Motor Groups
MotorGroup leftMotors({-14, 12, -13}, pros::MotorGearset::blue);
MotorGroup rightMotors({20, -19, 18}, pros::MotorGearset::blue);

//? Lemlib Odometry
pros::Imu imu(15);
Rotation LeftEnc(-11);
Rotation RightEnc(16);

lemlib::TrackingWheel LeftDrivetrain(
    &LeftEnc,
    lemlib::Omniwheel::NEW_325,
    -6.125,
    4.0/3.0
    );
lemlib::TrackingWheel RightDrivetrain(
    &RightEnc,
    lemlib::Omniwheel::NEW_325,
    6.125,
    4.0/3.0
    );  
lemlib::Drivetrain drivetrain(
	&leftMotors,
	&rightMotors,
	12.25,
	lemlib::Omniwheel::NEW_325,
	450,
	2
);

lemlib::OdomSensors sensors(
	&LeftDrivetrain,
    &RightDrivetrain,
	nullptr,
    nullptr,
    &imu
);

lemlib::ControllerSettings lateralController(
4.25,                     // proportional gain (kP)
    0.1,                      // integral gain (kI)
    2.5,                      // derivative gain (kD)
    2,             // anti windup
    .05, // small error range, in inches
    300, // small error range timeout, in milliseconds
    .2, // large error range, in inches
    700, // large error range timeout, in milliseconds
    1.25// maximum acceleration (slew)
);
lemlib::ControllerSettings angularController(
    2,                      // proportional gain (kP)
    0.25,                      // integral gain (kI)
    5,                     // derivative gain (kD)
    3,           // anti windup
    1, // small error range, in inches
    400, // small error range timeout, in milliseconds
    2, // large error range, in inches
    700, // large error range timeout, in milliseconds
    5                   // maximum acceleration (slew)
);



lemlib::ExpoDriveCurve throttleCurve(3, 7, 1.008);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.01);

lemlib::Chassis chassis(
    drivetrain,
    lateralController,
    angularController,
    sensors,
    &throttleCurve,
    &steerCurve
);