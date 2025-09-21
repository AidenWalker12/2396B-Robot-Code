#include "main.h" // IWYU pragma: keep
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "subsystems.hpp" // IWYU pragma: keep

void autonblueright() {
    // Reset pose
    chassis.setPose({0, 0, 0});

    // Drive to first ball
    chassis.moveToPoint(24, 0, 0);
    pros::delay(250);

    // Activate intake and drive to second ball
    intakeState = FORWARD;
    chassis.moveToPoint(48, 0, 0);
    pros::delay(250);

    // Drive to storage area
    chassis.moveToPoint(48, 24, 0);
    pros::delay(250);

    // Shoot preloaded balls
    beltState = FORWARD;
    pros::delay(2000);
    beltState = OFF;

    // Drive to alliance goal
    chassis.moveToPoint(24, 24, 0);
    pros::delay(250);

    // Deactivate intake
    intakeState = OFF;
}