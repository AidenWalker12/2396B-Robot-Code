#pragma once
#include "pros/motors.hpp"
#include "pros/motor_group.hpp"
#include "config.hpp"

// Extern motor objects
extern pros::Motor intake;
extern pros::Motor storage1;
extern pros::Motor storage2;
extern pros::Motor belt;
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

// State variables
extern State intakeState;
extern State storage1State;
extern State storage2State;
extern State beltState;

// Motor speed constants
extern const int INTAKE_SPEED;
extern const int STORAGE1_SPEED;
extern const int STORAGE2_SPEED;
extern const int BELT_SPEED;

// Functions
void updateMotorStates();
void setBrakeMode(pros::motor_brake_mode_e_t mode);
