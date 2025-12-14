#include "main.h"


//* Motor Groups
MotorGroup leftMotors({-11, 12, -13}, pros::MotorGearset::blue);
MotorGroup rightMotors({20, -19, 18}, pros::MotorGearset::blue);

//? Lemlib Odometry
Imu imu(15);
Rotation horizontalEnc(-16);
Rotation verticalEnc(-17);

lemlib::TrackingWheel horizontal(
						 &horizontalEnc,
				   lemlib::Omniwheel::NEW_2,
					    7.55
	);

lemlib::TrackingWheel vertical(
						&verticalEnc,
				  lemlib::Omniwheel::NEW_2,
					   .67
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
    10,                     // proportional gain (kP)
    0,                      // integral gain (kI)
    7,                      // derivative gain (kD)
    0,             // anti windup
    0,              // small error range, in inches
    0,     // small error range timeout, in milliseconds
    0,              // large error range, in inches
    0,     // large error range timeout, in milliseconds
    0                   // maximum acceleration (slew)
);
lemlib::ControllerSettings angularController(
    1.35,                      // proportional gain (kP)
    0,                      // integral gain (kI)
    8,                     // derivative gain (kD)
    0,           // anti windup
    0,             // small error range, in inches
    0,     // small error range timeout, in milliseconds
    0,              // large error range, in inches
    0,     // large error range timeout, in milliseconds
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