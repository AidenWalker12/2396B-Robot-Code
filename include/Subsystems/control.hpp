#pragma once
#include "pros/motors.hpp"
#include "pros/motor_group.hpp" 
#include "pros/adi.hpp"
//#include "pros/rtos.hpp"

// ---------- State Enum ----------
enum State { OFF = 0, FORWARD = 1, REVERSE = 2 };

// ---------- Constants -----------
const int INTAKE_SPEED   = 240;
const int STORAGE1_SPEED = 225;
const int STORAGE2_SPEED = 300;
const int BELT_SPEED     = 240;

// ---------- Extern Declarations ----------
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

extern pros::Motor intake;
extern pros::Motor storage1;
extern pros::Motor storage2;
extern pros::Motor belt;

extern pros::adi::Pneumatics scraper;

// ---------- State Vars ----------
extern State intakeState;
extern State storage1State;
extern State storage2State;
extern State beltState;
extern int config;

// ---------- Functions ----------
void initMotors();
void handleDrive(pros::Controller& master);
void handleMotorConfigs(pros::Controller& master);
void updateMotorStates();
