#include "subsystems/intake.hpp"

// Intake motor
pros::Motor intake(13, pros::MotorGearset::green);

void initIntake() {
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}
