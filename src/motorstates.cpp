#include "main.h"
#include "pros/motors.hpp"

//* Individual Motors
Motor firststage(-3  , pros::MotorGearset::blue); // USED INTAKE
Motor secondstage(-2, pros::MotorGearset::blue); //USED TOP
extern adi::Pneumatics hood;
// Global Motor States
State firststageState  = OFF;
State secondstageState = OFF;
State STOMP = OFF;
int config = 0;
int effectiveConfig = 0;

// Constraints
const int STOP         = 0;
int FIRSTSTAGE_SPEED   = 600;
int SECONDSTAGE_SPEED  = 600;
int stallCounterfirst = 0;
int stallCountersecond = 0;

//! Update Motor States 
void updateMotorStates() {
effectiveConfig = config;

//* Config System
    switch (effectiveConfig) {

        case NORUN:
            firststageState = OFF;
            secondstageState = OFF;
            break;

        case UP: // L1
            firststageState = FORWARD;
            secondstageState = FORWARD;
            break;

        case DOWN: // L2
            firststageState = REVERSE;
            secondstageState = REVERSE;
            break;

        case CENTERGOAL: // R1
            firststageState = FORWARD;
            secondstageState = REVERSE;
            break;
        
        case MODDOWN: // Special
            firststageState = REVERSE;
            secondstageState = REVERSE;
            break;
    }


        //* Stop the Overheating Motors Please(STOMP)

        // if (MasterController.get_digital(E_CONTROLLER_DIGITAL_L2) != false &&
        //     MasterController.get_digital(E_CONTROLLER_DIGITAL_L1) != false &&
        //     MasterController.get_digital(E_CONTROLLER_DIGITAL_R1) != false &&
        //     MasterController.get_digital(E_CONTROLLER_DIGITAL_R2) != false && 
        //     MasterController.get_digital(E_CONTROLLER_DIGITAL_A) != false) {
        //         STOMP = OFF;
        //         stallCounterfirst = 0;
        //         stallCountersecond = 0;
        // }
        
        // if (firststage.get_actual_velocity() < 10 && config != NORUN && hood.is_extended() == false && STOMP == OFF) {
        //         stallCounterfirst++;
        //         if(stallCounterfirst > 10) {
        //             firststageState = OFF;
        //             STOMP = FORWARD;
        //         }
        //     firststageState = OFF;
        // }
        // if (secondstage.get_actual_velocity() < 10 && config != NORUN && hood.is_extended() == false && STOMP == OFF) {
        //         stallCountersecond++;
        //         if(stallCountersecond > 10) {
        //             secondstageState = OFF;
        //             STOMP = FORWARD;
        //         }
        //     secondstageState = OFF;
        // }


    //* Apply motor velocities
    firststage.move_velocity(firststageState == FORWARD ? FIRSTSTAGE_SPEED :
                        firststageState == REVERSE ? -FIRSTSTAGE_SPEED : STOP);

    secondstage.move_velocity(secondstageState == FORWARD ? SECONDSTAGE_SPEED :
                        secondstageState == REVERSE ? -SECONDSTAGE_SPEED : STOP);
}