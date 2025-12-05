#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"


void updateMotorStates();
extern int config;
extern int GLPConfig;
extern lemlib::Chassis chassis;
Controller MasterController(pros::E_CONTROLLER_MASTER);
pros::adi::Pneumatics scraper('D', false, false);


//! Initialize 
void initialize() {
    pros::lcd::initialize();

    chassis.calibrate();

    intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    firststage.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    secondstage.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    pros::delay(500);


    pros::Task screenTask([]() {
        while (true) {
        lemlib::Pose pose = chassis.getPose();
        pros::lcd::print(0, "X: %.2f Y: %.2f Th: %.2f", pose.x, pose.y, pose.theta);
        pros::delay(100);
        }
    });
}

void disabled() {}
void competition_initialize() {}

void autonomous() {}
//! Operator Control 
void opcontrol() {

    while (true) {

        // Arcade drive 
        int leftY  = MasterController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = MasterController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX, false, 0.75);

        // Config toggles 
        if (MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) config = (config == UP)          ? NORUN : UP;
        if (MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) config = (config == DOWN)        ? NORUN : DOWN;
        if (MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) config = (config == CENTERGOAL)   ? NORUN : CENTERGOAL;

        updateMotorStates();
        if (MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        scraper.toggle();
        }

        pros::delay(10);
    }
}