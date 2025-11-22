#include "main.h"

//* Individual Motors
Motor intake(10, pros::MotorGearset::green);
Motor firststage(-1, pros::MotorGearset::green);
Motor secondstage(2, pros::MotorGearset::green);
//todo there will be more motors later

// Global Motor States
State intakeState   = OFF;
State firststageState = OFF;
State secondstageState = OFF;
int config = 0;
int effectiveConfig = 0;

// Constraints
const int INTAKE_SPEED   = 300;
const int FIRSTSTAGE_SPEED = 300;
const int SECONDSTAGE_SPEED = 300;

//! Update Motor States 
void updateMotorStates() {
effectiveConfig = config;
    //* Config System
    switch (effectiveConfig) {
        case NORUN:
            intakeState   = OFF;
            firststageState = OFF;
            secondstageState = OFF;
            break;

        case UP: // R1
            intakeState   = FORWARD;
            firststageState = FORWARD;
            secondstageState = FORWARD;
            break;

        case DOWN: // L1
            intakeState   = REVERSE;
            firststageState = REVERSE;
            secondstageState = REVERSE;
            break;

        case CENTERGOAL: // L2
            intakeState   = FORWARD;
            firststageState = FORWARD;
            secondstageState = REVERSE;
            break;

        
    }

    //* Apply motor velocities
    intake.move_velocity(intakeState == FORWARD ? INTAKE_SPEED :
                         intakeState == REVERSE ? -INTAKE_SPEED : 0);

    firststage.move_velocity(firststageState == FORWARD ? FIRSTSTAGE_SPEED :
                           firststageState == REVERSE ? -FIRSTSTAGE_SPEED : 0);

    secondstage.move_velocity(secondstageState == FORWARD ? SECONDSTAGE_SPEED :
                           secondstageState == REVERSE ? -SECONDSTAGE_SPEED : 0);

}