#include "main.h"


//* Motor Groups
MotorGroup leftMotors({-11, 12, -13}, pros::MotorGearset::blue);
MotorGroup rightMotors({20, -19, 18}, pros::MotorGearset::blue);

//? Lemlib Odometry
Imu imu(25);
Rotation horizontalEnc(26);
Rotation verticalEnc(27);

lemlib::TrackingWheel horizontal(
						 &horizontalEnc,
				   lemlib::Omniwheel::NEW_2,
					    7.0
	);

lemlib::TrackingWheel vertical(
						&verticalEnc,
				  lemlib::Omniwheel::NEW_2,
					   -.5
	);

lemlib::Drivetrain drivetrain(
				   &leftMotors,
				  &rightMotors,
				   12.5,
				lemlib::Omniwheel::NEW_325,
				          450,
			  1
);

lemlib::ControllerSettings lateralController(
    8,                     // proportional gain (kP)
    1,                      // integral gain (kI)
    5,                      // derivative gain (kD)
    3,             // anti windup
    .5,              // small error range, in inches
    100,     // small error range timeout, in milliseconds
    1.5,              // large error range, in inches
    500,     // large error range timeout, in milliseconds
    127                   // maximum acceleration (slew)
);
lemlib::ControllerSettings angularController(
    4,                      // proportional gain (kP)
    0,                      // integral gain (kI)
    13,                     // derivative gain (kD)
    5.5,           // anti windup
    .3,             // small error range, in inches
    200,     // small error range timeout, in milliseconds
    .5,              // large error range, in inches
        1000,     // large error range timeout, in milliseconds
    0                     // maximum acceleration (slew)
);

lemlib::OdomSensors sensors(
							&vertical,
							nullptr,
						  &horizontal,
					  	  nullptr,
								  &imu
								);

lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

lemlib::Chassis chassis(
                        drivetrain,
        lateralController,
       angularController,
                        sensors,
         &throttleCurve,
            &steerCurve
);