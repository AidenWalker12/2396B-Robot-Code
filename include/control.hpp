#pragma once
#include "main.h" // IWYU pragma: keep
#include "config.hpp"

// Global controller
extern pros::Controller Controller;

// Motor states
extern State intakeState;
extern State storage1State;
extern State storage2State;
extern State beltState;
extern int config;

// Update motor states based on controller & config
void updateMotorStates();

