#include "main.h"


//* Motor Groups
MotorGroup leftMotors({-14, 12, -13}, pros::MotorGearset::blue);
MotorGroup rightMotors({20, -19, 18}, pros::MotorGearset::blue);

//? Lemlib Odometry
Imu imu(15);
Rotation horizontalEnc(-16);
Rotation verticalEnc(17);
pros::GPS gps(25);

lemlib::TrackingWheel horizontal(
						 &horizontalEnc,
				   lemlib::Omniwheel::NEW_2,
					  -7.625
	);

lemlib::TrackingWheel vertical(
						&verticalEnc,
				  lemlib::Omniwheel::NEW_2,
             .5
	);

lemlib::Drivetrain drivetrain(
	&leftMotors,
	&rightMotors,
	12.25,
	lemlib::Omniwheel::NEW_325,
	450,
	7
);

lemlib::ControllerSettings lateralController(
    4.3,                     // proportional gain (kP)
    0,                      // integral gain (kI)
    7,                      // derivative gain (kD)
    3,             // anti windup
    .1,              // small error range, in inches
    50,     // small error range timeout, in milliseconds
    .1,              // large error range, in inches
    250,     // large error range timeout, in milliseconds
    0                   // maximum acceleration (slew)
);
lemlib::ControllerSettings angularController(
    1,                      // proportional gain (kP)
    0,                      // integral gain (kI)
    8,                     // derivative gain (kD)
    3,           // anti windup
    .1,             // small error range, in inches
    50,     // small error range timeout, in milliseconds
    .1,              // large error range, in inches
    250,     // large error range timeout, in milliseconds
    0                    // maximum acceleration (slew)
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