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
    // scraper.set_value(false);
    // config = UP;
    // updateMotorStates();
    // chassis.moveToPose(-0.37, 30.5, 0.67, 2500, {.maxSpeed = 70});
    // pros::delay(2000);
    // config = NORUN;
    // updateMotorStates();
    // chassis.turnToHeading(-77, 900,{.maxSpeed = 63});
    // chassis.moveToPoint(-11.8, 36.1, 1000);
    // pros::delay(1000);
    // config = DOWN;
    // updateMotorStates();
    // pros::delay( 1000);
    // config = UP;
    // pros::delay( 400);
    // config = NORUN;
    // updateMotorStates();
    // chassis.moveToPoint(32.98, 11.55, 2000,{.forwards = false});
    // updateMotorStates();
    // chassis.turnToHeading(-200,800, {.maxSpeed = 63});
    // scraper.set_value(true);
    // config = UP;
    // updateMotorStates();
    // chassis.moveToPose(36.4, 5.4, -202, 1300);
    // chassis.moveToPoint(36.4, -12, 1500, {.maxSpeed = 40});
    // chassis.moveToPoint(29, 23.4, 1000,{.forwards = false, .minSpeed = 70});
    // chassis.moveToPoint(28.7, 36, 2500, {.forwards = false, .minSpeed = 100});
    //     hood.set_value(true);
}
void BlueRight() {


}
void BlueLeft() {

}