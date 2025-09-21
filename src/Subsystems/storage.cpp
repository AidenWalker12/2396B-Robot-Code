#include "subsystems/storage.hpp"

// Storage motors
pros::Motor storage1(-3, pros::MotorGearset::green);
pros::Motor storage2(14, pros::MotorGearset::green);

void initStorage() {
    storage1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    storage2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}
