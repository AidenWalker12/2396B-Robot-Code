#pragma once
#include "pros/misc.hpp"
#include "pros/motors.hpp"

// Declare functions
void updateMotorStates();
void handleBeltControl(pros::Controller& master);
void handleBrakeModeCycle(pros::Controller& master);

// Declare motors so they are accessible in auton
extern pros::Motor beltMotor;
extern pros::Motor storageMotor;
