#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/ai_vision.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"
#include "pros/vision.hpp"
#include <math.h>

//? Controller
using pros::AIVision;
using pros::Imu;
using pros::Motor;
using pros::MotorGroup;
using pros::Rotation;
using pros::Vision;
using pros::adi::DigitalOut;
using pros::adi::Pneumatics;
using pros::adi::Potentiometer;

pros::Controller Controller(pros::E_CONTROLLER_MASTER);

//? Motor State Enum 
enum State { OFF = 0, FORWARD = 1, REVERSE = 2 };

//? Motor config Enum
enum Config { NORUN = 0, UP = 1, DOWN = 2, STORAGEIN = 3, STORAGELONG = 4, UPMID = 5, STORAGEDOWN = 6, STORAGEMID = 7,  };

//? GLP Enum
enum GLPConfig {FULLOFF = 0, SCRAPERON = 1, GLPON = 2};

//? Team Enum
enum Team {Default = 0, Red = 1, Blue = 2};


//? Global Subsystems
    //* ADI Ports
DigitalOut scraperDigital(1);
AIVision ColorSort(20);

Pneumatics scraper('A', false, false);
Pneumatics glp('C', true, false);

Potentiometer autosensor('B', pros::E_ADI_POT_EDR);

//* Motor Groups
MotorGroup leftMotors({-1, -2}, pros::MotorGearset::blue);
MotorGroup rightMotors({11, 12}, pros::MotorGearset::blue);

//* Individual Motors
Motor intake(13, pros::MotorGearset::green);
Motor storage1(-3, pros::MotorGearset::green);
Motor storage2(14, pros::MotorGearset::green);
Motor belt(15, pros::MotorGearset::green);

//? Lemlib Odometry
Imu imu(16);
Rotation horizontalEnc(17);
Rotation verticalEnc(-18);

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
				   11.75,
				lemlib::Omniwheel::NEW_275,
				          600,
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

// Constraints
const int INTAKE_SPEED   = 240;
const int STORAGE1_SPEED = 225;
const int STORAGE2_SPEED = 300;
const int BELT_SPEED     = 240;

//autosensor


// Global Motor States
State intakeState   = OFF;
State storage1State = OFF;
State storage2State = OFF;
State beltState     = OFF;
int Team = Default;
int config = 0;
int AutoR = 0;
int GLPConfig = 0;
int effectiveConfig = 0;
int effectiveTeam = 0;

// Color Sort
bool ColorSortRan = false;

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

    pros::delay(500);


    pros::Task screenTask([]() {
        while (true) {
        lemlib::Pose pose = chassis.getPose();
float angle = (float)autosensor.get_value() * (270.0 / 4095.0); // for V1 270 rotation
        pros::lcd::print(0, "X: %.2f Y: %.2f Th: %.2f", pose.x, pose.y, pose.theta);
        pros::lcd::print(1, "Angle: %.1f", angle);
        pros::delay(100);
        }
    });
}

void disabled() {}
void competition_initialize() {}

//! Update Motor States 
void updateMotorStates() {
    if(ColorSortRan == true) {
        config = effectiveConfig;
    }
    else if(ColorSortRan == false) {
        effectiveConfig = config; // baseline
    }


    // Overrides
	if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y) && config == STORAGELONG) { 
		effectiveConfig = STORAGEMID; // belt
	}
 	if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y) && config == UP) { 
		effectiveConfig = 8; // BELT
	}   
    if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) && config == STORAGELONG) {
        effectiveConfig = STORAGEDOWN;  // special belt reverse
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

        case STORAGEDOWN: // L2 + Right
            intakeState   = REVERSE;
            storage1State = REVERSE;
            storage2State = FORWARD;
            beltState     = OFF;
            break;

		case STORAGEMID: // L2 + Y
			intakeState = FORWARD;
			storage1State = REVERSE;
			storage2State = FORWARD;
			beltState = REVERSE;
			break;
        
        case 8:
        	intakeState = FORWARD;
			storage1State = FORWARD;
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


void GLP() {
    switch (GLPConfig) {
        case FULLOFF:
            scraper.set_value(false);
            glp.set_value(true);
            break;
        case SCRAPERON:
            scraper.set_value(true);
            glp.set_value(true);
            break;
        case GLPON:
            scraper.set_value(false);
            glp.set_value(false);
            break;
    }

}

void Alliance() {
    effectiveTeam = Team;
    //? Alliance Color
        float angle = (float)autosensor.get_value() * (270.0 / 4095.0); // for V1 270 rotation

        if(angle >= 0 && angle  <= 30 ) {
            pros::lcd::print(3, "Left Red");
            Team = Red;
        }
        else if(angle > 30 && angle <= 125) {
            pros::lcd::print(3, "Left Blue");
            Team = Blue;
        }
        else if(angle > 125 && angle <= 240) {
            pros::lcd::print(3, "Right Blue");
            Team = Blue;
        }
        else if(angle > 240 && angle <= 270) {
            pros::lcd::print(3, "Right Red");
            Team = Red;
        }

}

void RedLeft() {
    // chassis.setPose(0,0,0);
    // chassis.moveToPoint(0, 13.5, 2500);
    // chassis.turnToHeading(-41.45, 1000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .earlyExitRange = 5});
    // config = UP;
    // updateMotorStates();
    // chassis.moveToPoint(-.97, 27.74, 2000);
    // chassis.turnToHeading(45,500, {.direction = AngularDirection::CW_CLOCKWISE});
    // pros::delay(500);
    // config = NORUN;
    // updateMotorStates();
    // chassis.turnToHeading(45,200, {.direction = lemlib::AngularDirection::CW_CLOCKWISE});
    // chassis.moveToPoint(-8.38, 40.56, 1000, {.maxSpeed = 300});
    // chassis.turnToHeading(47, 400);
    // pros::delay(500);
    // config = UPMID;
    // updateMotorStates();
    // pros::delay(1000);
    // config = UP;
    // updateMotorStates();
    // pros::delay(500);
    // config = NORUN;
    // updateMotorStates();
}

void BlueLeft()  {

}

void BlueRight() {

}
void RedRight()  {
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0, 13.5, 2500, {.maxSpeed = 300});
    chassis.turnToHeading(41.45, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .earlyExitRange = 5});
    config = UP;
    updateMotorStates();
    chassis.moveToPoint(.97, 27.74, 2000);
    chassis.turnToHeading(0,500);
    chassis.turnToHeading(30,500);
    config = NORUN;
    updateMotorStates();
    chassis.moveToPoint(8, 42, 4000);
    chassis.turnToHeading(-45, 400);


    config = DOWN;
    updateMotorStates();
    pros::delay(1000);
    config = UP;
    updateMotorStates();
    pros::delay(500);
    config = NORUN;
    updateMotorStates();
    // chassis.moveToPoint(0,0,2000, {.forwards = false});
    // chassis.turnToHeading(180,1000);
}
void autonomous() {


    float angle = (float)autosensor.get_value() * (270.0 / 4095.0); // for V1 270 rotation
    if(angle >= 0 && angle  <= 30 ) {
        pros::lcd::print(3, "Left Red");
        RedLeft();
    }
    else if(angle > 30 && angle <= 125) {
        pros::lcd::print(3, "Left Blue");
        BlueLeft();
    }
    else if(angle > 125 && angle <= 240) {
        pros::lcd::print(3, "Right Blue");
        BlueRight();
    }
    else if(angle > 240 && angle <= 270) {
        pros::lcd::print(3, "Right Red");
        RedRight();
    }

}
//! Operator Control 
void opcontrol() {

    //! Alliance Color
    // Alliance();
    // Resetting the motor states for Operator Control
    State intakeState   = OFF;
    State storage1State = OFF;
    State storage2State = OFF;
    State beltState     = OFF;
    
    while (true) {

        // Arcade drive 
        int leftY  = Controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = Controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX, false, 0.75);

    
        //? Config toggles 
        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) config = (config == UP)          ? NORUN : UP;
        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) config = (config == DOWN)        ? NORUN : DOWN;
        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) config = (config == STORAGEIN)   ? NORUN : STORAGEIN;
        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) config = (config == STORAGELONG) ? NORUN : STORAGELONG;

        //? Gem Loss Protection
        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) GLPConfig = (GLPConfig == SCRAPERON) ? FULLOFF : SCRAPERON;
        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) GLPConfig = (GLPConfig == GLPON) ? FULLOFF : GLPON;

        //? Emergency Shutoff for Color Sort
        if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) &&
            Controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) &&
            Controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) &&
            Controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            Team = (Team == Default) ? effectiveTeam : Default;
            effectiveTeam = Team;
            }
        //! Update motors 
        updateMotorStates();
        
        //! Update GLP
        GLP();



        // //? Color Sort
        // if (Team == Red /* && vision detected == blue */) {
        //     //todo config = DOWN;
        //     //todo updateMotorStates();
        //     //todo when sight ends
        //     //todo ColorSortRan = True;
        //     //todo updateMotorStates();
        //     //todo }
        // }
        // else if (Team == Blue /* && vision detected == red */) {
        //     //todo config = DOWN;
        //     //todo updateMotorStates();
        //     //todo when sight ends
        //     //todo ColorSortRan = True;
        //     //todo updateMotorStates();
        //     //todo }
        // }


        pros::delay(10);
    }
}