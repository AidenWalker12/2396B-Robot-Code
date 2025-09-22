#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"
#include <math.h>

//? Controller 
pros::Controller Controller(pros::E_CONTROLLER_MASTER);

//? Motor State Enum 
enum State { OFF = 0, FORWARD = 1, REVERSE = 2 };
//? Motor config Enum
enum Config { NORUN = 0, UP = 1, DOWN = 2, STORAGEIN = 3, STORAGELONG = 4, UPMID = 5, STORAGEDOWN = 6, STORAGEMID = 7 };

//? Global Subsystems
pros::adi::DigitalOut scraperDigital(1);
pros::adi::Pneumatics scraper('A', false, false);

pros::MotorGroup leftMotors({-1, -2}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({11, 12}, pros::MotorGearset::blue);

pros::Motor intake(13, pros::MotorGearset::green);
pros::Motor storage1(-3, pros::MotorGearset::green);
pros::Motor storage2(14, pros::MotorGearset::green);
pros::Motor belt(15, pros::MotorGearset::green);

//? Lemlib Odometry 
pros::Imu imu(16);
pros::Rotation horizontalEnc(17);
pros::Rotation verticalEnc(18);

lemlib::TrackingWheel horizontal(
						 &horizontalEnc,
				   lemlib::Omniwheel::NEW_2,
					    -6.7
	);

lemlib::TrackingWheel vertical(
						&verticalEnc,
				  lemlib::Omniwheel::NEW_2,
					   -0.5
	);

lemlib::Drivetrain drivetrain(
				   &leftMotors,
				  &rightMotors,
				   10.896752,
				lemlib::Omniwheel::NEW_275,
				          600,
			  6
);

lemlib::ControllerSettings linearController(
										 10,  // proportional gain (kP)
										 0,   // integral gain (kI)
										 3,   // derivative gain (kD)
								0,   // anti windup
								 0,   // small error range, in inches
						  0, // small error range timeout, in milliseconds
								 0,   // large error range, in inches
						  0, // large error range timeout, in milliseconds
									   0   // maximum acceleration (slew)
		);
lemlib::ControllerSettings angularController(
										  2,   // proportional gain (kP)
										  0,   // integral gain (kI)
										  10,  // derivative gain (kD)
								 0,   // anti windup
							 	  0,   // small error range, in inches
						   0, // small error range timeout, in milliseconds
							      0,   // large error range, in inches
						   0, // large error range timeout, in milliseconds
									    0    // maximum acceleration (slew)
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
    linearController,
    angularController,
    sensors,
    &throttleCurve,
    &steerCurve
);

// Constraints
const int INTAKE_SPEED   = 240;
const int STORAGE1_SPEED = 225;
const int STORAGE2_SPEED = 300;
const int BELT_SPEED     = 240;

// Global Motor States
State intakeState   = OFF;
State storage1State = OFF;
State storage2State = OFF;
State beltState     = OFF;
int config = 0;

// Brake Mode 
pros::motor_brake_mode_e brakeMode = pros::E_MOTOR_BRAKE_COAST;

//! Initialize 
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();

    intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    storage1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    storage2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    belt.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    pros::Task screenTask([]() {
        while (true) {
            lemlib::Pose pose = chassis.getPose();
            pros::lcd::print(0, "X: %.2f Y: %.2f Th: %.2f", pose.x, pose.y, pose.theta);
            pros::delay(100);
        }
    });
}

void disabled() {}
void competition_initialize() {}

//! Update Motor States 
void updateMotorStates() {
    int effectiveConfig = config; // baseline

    //* Overrides
    if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y) && config == STORAGELONG) {
        effectiveConfig = STORAGEDOWN;  // special belt reverse
    }
    if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && config == UP) {
        effectiveConfig = UPMID;  // special storage/belt pattern
    }
	if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && config == STORAGELONG) { 
		effectiveConfig = STORAGEMID; // intake only
	}		
    //* Config System
    switch (effectiveConfig) {
        case NORUN:
            intakeState   = OFF;
            storage1State = OFF;
            storage2State = OFF;
            beltState     = OFF;
            break;

        case UP: // R1
            intakeState   = FORWARD;
            storage1State = FORWARD;
            storage2State = FORWARD;
            beltState     = FORWARD;
            break;

        case DOWN: // R2
            intakeState   = REVERSE;
            storage1State = REVERSE;
            storage2State = REVERSE;
            beltState     = REVERSE;
            break;

        case STORAGEIN: // L1
            intakeState   = FORWARD;
            storage1State = FORWARD;
            storage2State = REVERSE;
            beltState     = OFF;
            break;

        case STORAGELONG: // L2
            intakeState   = FORWARD;
            storage1State = REVERSE;
            storage2State = FORWARD;
            beltState     = FORWARD;
            break;

        case UPMID: // R1 + Down
            intakeState   = FORWARD;
            storage1State = FORWARD;
            storage2State = FORWARD;
            beltState     = REVERSE;
            break;

        case STORAGEDOWN: // L2 + Y
            intakeState   = REVERSE;
            storage1State = REVERSE;
            storage2State = FORWARD;
            beltState     = OFF;
            break;

		case STORAGEMID: // L2 + Down
			intakeState = FORWARD;
			storage1State = REVERSE;
			storage2State = FORWARD;
			beltState = REVERSE;
			break;
    }

    //* Apply motor velocities
    intake.move_velocity(intakeState == FORWARD ? INTAKE_SPEED :
                         intakeState == REVERSE ? -INTAKE_SPEED : 0);

    storage1.move_velocity(storage1State == FORWARD ? STORAGE1_SPEED :
                           storage1State == REVERSE ? -STORAGE1_SPEED : 0);

    storage2.move_velocity(storage2State == FORWARD ? STORAGE2_SPEED :
                           storage2State == REVERSE ? -STORAGE2_SPEED : 0);

    belt.move_velocity(beltState == FORWARD ? BELT_SPEED :
                       beltState == REVERSE ? -BELT_SPEED : 0);
}

void autonomous() {
	// Example autonomous routine
	leftMotors.move_velocity(200);
	rightMotors.move_velocity(200);
	pros::delay(1000); // for 1 seconds
	leftMotors.move_velocity(0);
	rightMotors.move_velocity(0); // Stop
}
//! Operator Control 
void opcontrol() {
    while (true) {
        // Arcade drive 
        int leftY  = Controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = Controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX, false, 0.75);

        //? Config toggles 
        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) config = (config == UP)         ? NORUN : UP;
        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) config = (config == DOWN)       ? NORUN : DOWN;
        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) config = (config == STORAGEIN)  ? NORUN : STORAGEIN;
        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) config = (config == STORAGELONG) ? NORUN : STORAGELONG;

        //! Update motors 
        updateMotorStates();

        // Pneumatic toggle 
        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            scraper.toggle();
        }

        pros::delay(10);
    }
}