#pragma once
#include "pros/motors.hpp" // IWYU pragma: keep

// Motor speeds
constexpr int INTAKE_SPEED   = 240;
constexpr int STORAGE1_SPEED = 225;
constexpr int STORAGE2_SPEED = 300;
constexpr int BELT_SPEED     = 240;

// Brake mode
constexpr auto BRAKE_COAST = pros::E_MOTOR_BRAKE_COAST;
constexpr auto BRAKE_BRAKE = pros::E_MOTOR_BRAKE_BRAKE;

// Motor states
enum State { OFF = 0, FORWARD = 1, REVERSE = 2 };

// Motor configurations
enum config {
    NORUN        = 0,
    UP           = 1,
    DOWN         = 2,
    STORAGEIN    = 3,
    STORAGELONG  = 4,
    UPMID        = 5,
    STORAGEDOWN  = 6,
    STORAGEMID   = 7
};
