#include "Subsystems/motors.hpp"
#include "Subsystems/config.hpp"

// Motor definitions
pros::MotorGroup leftMotors({-1, -2}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({11, 12}, pros::MotorGearset::blue);

pros::Motor intake(13, pros::MotorGearset::green);
pros::Motor storage1(-3, pros::MotorGearset::green);
pros::Motor storage2(14, pros::MotorGearset::green);
pros::Motor belt(15, pros::MotorGearset::green);

// Speeds
const int INTAKE_SPEED   = 240;
const int STORAGE1_SPEED = 225;
const int STORAGE2_SPEED = 300;
const int BELT_SPEED     = 240;

// Global states
State intakeState   = OFF;
State storage1State = OFF;
State storage2State = OFF;
State beltState     = OFF;

// Motor States


void updateMotorStates() {
    int effectiveConfig = config;

    // Overrides
    if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y) && config == STORAGELONG) {
        effectiveConfig = STORAGEDOWN;
    }
    if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && config == UP) {
        effectiveConfig = UPMID;
    }
    if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && config == STORAGELONG) {
        effectiveConfig = STORAGEMID;
    }

    // Config switch
    switch (effectiveConfig) {
        case NORUN:
            intakeState = storage1State = storage2State = beltState = OFF;
            break;

        case UP:
            intakeState = storage1State = storage2State = beltState = FORWARD;
            break;

        case DOWN:
            intakeState = storage1State = storage2State = beltState = REVERSE;
            break;

        case STORAGEIN:
            intakeState = FORWARD; storage1State = FORWARD;
            storage2State = REVERSE; beltState = OFF;
            break;

        case STORAGELONG:
            intakeState = FORWARD; storage1State = REVERSE;
            storage2State = FORWARD; beltState = FORWARD;
            break;

        case UPMID:
            intakeState = FORWARD; storage1State = FORWARD;
            storage2State = FORWARD; beltState = REVERSE;
            break;

        case STORAGEDOWN:
            intakeState = REVERSE; storage1State = REVERSE;
            storage2State = FORWARD; beltState = OFF;
            break;

        case STORAGEMID:
            intakeState = FORWARD; storage1State = REVERSE;
            storage2State = FORWARD; beltState = REVERSE;
            break;
    }

    // Apply motor commands
    intake.move_velocity(intakeState == FORWARD ? INTAKE_SPEED :
                         intakeState == REVERSE ? -INTAKE_SPEED : 0);

    storage1.move_velocity(storage1State == FORWARD ? STORAGE1_SPEED :
                           storage1State == REVERSE ? -STORAGE1_SPEED : 0);

    storage2.move_velocity(storage2State == FORWARD ? STORAGE2_SPEED :
                           storage2State == REVERSE ? -STORAGE2_SPEED : 0);

    belt.move_velocity(beltState == FORWARD ? BELT_SPEED :
                       beltState == REVERSE ? -BELT_SPEED : 0);
}

void setBrakeMode(pros::motor_brake_mode_e_t mode) {
    intake.set_brake_mode(mode);
    storage1.set_brake_mode(mode);
    storage2.set_brake_mode(mode);
    belt.set_brake_mode(mode);
    leftMotors.set_brake_mode(mode);
    rightMotors.set_brake_mode(mode);
}
