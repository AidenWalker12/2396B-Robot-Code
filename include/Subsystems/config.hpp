#pragma once
#include "pros/misc.hpp"

// Controller
extern pros::Controller Controller;

// Motor states
enum State { OFF = 0, FORWARD = 1, REVERSE = 2 };

// Motor configs
enum Config { 
    NORUN = 0, 
    UP = 1, 
    DOWN = 2, 
    STORAGEIN = 3, 
    STORAGELONG = 4, 
    UPMID = 5, 
    STORAGEDOWN = 6, 
    STORAGEMID = 7 
};

// Global config variable
extern int config;