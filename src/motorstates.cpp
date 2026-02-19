#include "main.h"
#include "pros/motors.hpp"

//* Individual Motors
Motor intake(21, pros::MotorGearset::blue);
Motor firststage(-3  , pros::MotorGearset::blue);
Motor secondstage(-2, pros::MotorGearset::blue);
Motor top(8, pros::MotorGearset::blue);

// Global Motor States
State intakeState      = OFF;
State firststageState  = OFF;
State secondstageState = OFF;
State topState         = OFF;
int config = 0;
int effectiveConfig = 0;

// Constraints
const int STOP         = 0;
int INTAKE_SPEED = 600;
int TOP_SPEED    = 600;
int FIRSTSTAGE_SPEED   = 600;
int SECONDSTAGE_SPEED  = 600;

//! Update Motor States 
void updateMotorStates() {
effectiveConfig = config;

//* Config System
    switch (effectiveConfig) {

        case NORUN:
            intakeState   = OFF;
            firststageState = OFF;
            secondstageState = OFF;
            topState = OFF;
            break;

        case UP: // L1
            intakeState   = FORWARD;
            firststageState = FORWARD;
            secondstageState = FORWARD;
            topState = FORWARD;
            break;

        case DOWN: // L2
            intakeState   = REVERSE;
            firststageState = REVERSE;
            secondstageState = REVERSE;
            topState = REVERSE;
            break;

        case CENTERGOAL: // R1
            intakeState   = FORWARD;
            firststageState = FORWARD;
            secondstageState = REVERSE;
            topState = REVERSE;
            break;
        
        case MODDOWN: // Special
            intakeState   = REVERSE;
            firststageState = REVERSE;
            secondstageState = REVERSE;
            topState = OFF;
            break;

        
    }

    //* Apply motor velocities
    intake.move_velocity(intakeState == FORWARD ? INTAKE_SPEED :
                        intakeState == REVERSE ? -INTAKE_SPEED : STOP);

    firststage.move_velocity(firststageState == FORWARD ? FIRSTSTAGE_SPEED :
                        firststageState == REVERSE ? -FIRSTSTAGE_SPEED : STOP);

    secondstage.move_velocity(secondstageState == FORWARD ? SECONDSTAGE_SPEED :
                        secondstageState == REVERSE ? -SECONDSTAGE_SPEED : STOP);

    top.move_velocity(topState == FORWARD ? TOP_SPEED :
                        topState == REVERSE ? -TOP_SPEED : STOP); 

}