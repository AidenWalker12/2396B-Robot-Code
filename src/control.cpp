#include "subsystems.hpp" // IWYU pragma: keep
#include "main.h" // IWYU pragma: keep

// Controller
pros::Controller Controller(pros::E_CONTROLLER_MASTER);

// Motor states
State intakeState = OFF;
State storage1State = OFF;
State storage2State = OFF;
State beltState = OFF;
int Config = NORUN;

// Autonomous Selector
int autonmode = SKILLS;

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

    // Config system: assign each motor separately for clarity
    switch(effectiveConfig) {
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
            intakeState   = FORWARD;
            storage1State = REVERSE;
            storage2State = FORWARD;
            beltState     = REVERSE;
            break;
    }

    // Apply motor velocities separately, formatted for readability
    intake.move_velocity(
        (intakeState == FORWARD)  ?  INTAKE_SPEED  :
        (intakeState == REVERSE)  ? -INTAKE_SPEED  : 0
    );

    storage1.move_velocity(
        (storage1State == FORWARD)  ?  STORAGE1_SPEED  :
        (storage1State == REVERSE)  ? -STORAGE1_SPEED  : OFF
    );

    storage2.move_velocity(
        (storage2State == FORWARD)  ?  STORAGE2_SPEED  :
        (storage2State == REVERSE)  ? -STORAGE2_SPEED  : OFF
    );

    belt.move_velocity(
        (beltState == FORWARD)  ?  BELT_SPEED  :
        (beltState == REVERSE)  ? -BELT_SPEED  : OFF
    );
}
