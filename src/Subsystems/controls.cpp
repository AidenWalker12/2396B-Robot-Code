#include "Subsystems/control.hpp"
#include "Subsystems/motors.hpp"
#include "pros/motors.hpp"
#include "pros/misc.hpp"

// -------------------- Motor Declarations -------------------- //
pros::Motor beltMotor(1); 
pros::Motor storageMotor(2);

// -------------------- State Variables -------------------- //
bool beltForward = false;
bool beltReverse = false;
int brakeModeIndex = 0;

// -------------------- Functions -------------------- //
void updateMotorStates() {
    if (beltForward) {
        beltMotor.move_velocity(600);
    } else if (beltReverse) {
        beltMotor.move_velocity(-600);
    } else {
        beltMotor.move_velocity(0);
    }
}

void handleBeltControl(pros::Controller& master) {
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
        beltForward = !beltForward;
        beltReverse = false;
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
        beltReverse = !beltReverse;
        beltForward = false;
    }
}

void handleBrakeModeCycle(pros::Controller& master) {
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        brakeModeIndex = (brakeModeIndex + 1) % 3;

        pros::motor_brake_mode_e_t mode;
        switch (brakeModeIndex) {
            case 0: mode = pros::E_MOTOR_BRAKE_COAST; break;
            case 1: mode = pros::E_MOTOR_BRAKE_BRAKE; break;
            case 2: mode = pros::E_MOTOR_BRAKE_HOLD; break;
        }

        beltMotor.set_brake_mode(mode);
        storageMotor.set_brake_mode(mode);
    }
}
