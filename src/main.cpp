#include "pros/llemu.hpp"
#include "Subsystems/motors.hpp"
#include "Subsystems/odometry.hpp"

void initialize() {
    pros::lcd::initialize();
    initOdometry();

    setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    // Debug task
    pros::Task screenTask([]() {
        while (true) {
            lemlib::Pose pose = chassis.getPose();
            pros::lcd::print(0, "X: %.2f Y: %.2f Th: %.2f", pose.x, pose.y, pose.theta);
            pros::delay(100);
            initMotors();  // sets brake modes and motor init

        }
    });
}

void disabled() {}
void competition_initialize() {}
