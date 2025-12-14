#include "main.h"

extern lemlib::Chassis chassis;

void RedLeft() {
    chassis.setPose(0,0,0);
    chassis.moveToPoint(-.43, 13, 3000);
    chassis.turnToHeading(-44.68, 1000);
    chassis.moveToPoint(0, 13.5, 2500);
    chassis.turnToHeading(-41.45, 1000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .earlyExitRange = 5});
    config = UP;
    updateMotorStates();
    chassis.moveToPoint(-.97, 28.24, 2000);
    chassis.turnToHeading(39.55, 1000);
    chassis.moveToPoint(-9.04, 40, 2000);
    chassis.turnToHeading(44.08, 1000);
    chassis.moveToPoint(-.97, 27.74, 2000);
    chassis.turnToHeading(45,500, {.direction = AngularDirection::CW_CLOCKWISE});
    pros::delay(500);
    config = NORUN;
    updateMotorStates();
    chassis.turnToHeading(45,200, {.direction = lemlib::AngularDirection::CW_CLOCKWISE});
    chassis.moveToPoint(-8.38, 40.56, 1000, {.maxSpeed = 300});
    chassis.turnToHeading(47, 400);
    pros::delay(500);
    config = CENTERGOAL;
    updateMotorStates();
    pros::delay(1000);
    config = DOWN;
    config = UP;
    updateMotorStates();
    pros::delay(2000);
    pros::delay(500);
    config = NORUN;
    updateMotorStates();
}
void RedRight() {
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0, 13.5, 2500);
    chassis.moveToPoint(0, 13.5, 2500, {.maxSpeed = 300});
    chassis.turnToHeading(41.45, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .earlyExitRange = 5});
    config = UP;
    updateMotorStates();
    chassis.moveToPoint(.97, 27.74, 2000);
    chassis.turnToHeading(-45,500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
    pros::delay(500);
    chassis.turnToHeading(0,500);
    chassis.turnToHeading(30,500);
    config = NORUN;
    updateMotorStates();
    chassis.turnToHeading(-45,200, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(8.38, 40.56, 1000, {.maxSpeed = 300});
    chassis.turnToHeading(-47, 400);
    pros::delay(500);
    chassis.moveToPoint(8, 42, 4000);
    chassis.turnToHeading(-45, 400);
    config = DOWN;
    updateMotorStates();
    pros::delay(1000);
    config = UP;
    updateMotorStates();
    pros::delay(500);
    config = NORUN;
    updateMotorStates();
    chassis.moveToPoint(0,0,2000, {.forwards = false});
    chassis.turnToHeading(180,1000);
}
void BlueRight() {


}
void BlueLeft() {

}