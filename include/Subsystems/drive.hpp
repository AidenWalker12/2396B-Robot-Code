#pragma once
#include "pros/motors.hpp" // IWYU pragma: keep
#include "pros/motor_group.hpp"
#include "pros/imu.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"

// Drive motors
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

// IMU and encoders
extern pros::Imu imu;
extern pros::Rotation horizontalEnc;
extern pros::Rotation verticalEnc;

// Lemlib objects
extern lemlib::TrackingWheel horizontal;
extern lemlib::TrackingWheel vertical;
extern lemlib::Drivetrain drivetrain;
extern lemlib::Chassis chassis;

// Initialize drivetrain & set brake modes
void initDrive();
