#include "subsystems/belt.hpp"

// Belt motor
pros::Motor belt(15, pros::MotorGearset::green);

void initBelt() {
    belt.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}
