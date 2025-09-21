#include "main.h"
#include "subsystems.hpp" // IWYU pragma: keep
#include "lemlib/api.hpp" // IWYU pragma: keep


void initialize() {
    pros::lcd::initialize();
    initDrive();
    initIntake();
    initStorage();
    initBelt();
    initPneumatics();

    pros::Task screenTask([](){
        while(true) {
            lemlib::Pose pose = chassis.getPose();
            pros::lcd::print(0, "X: %.2f Y: %.2f Th: %.2f", pose.x, pose.y, pose.theta);
            pros::delay(100);
        }
    });
}

void disabled() {}
void competition_initialize() {}

void autonomous() {
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 100000);
}

void opcontrol() {
    while(true) {
        int leftY = Controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = Controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX, false, 0.75);

        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
            config = (config == UP) ? NORUN : UP;
        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2))
            config = (config == DOWN) ? NORUN : DOWN;
        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
            config = (config == STORAGEIN) ? NORUN : STORAGEIN;
        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
            config = (config == STORAGELONG) ? NORUN : STORAGELONG;

        updateMotorStates();

        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
            scraper.toggle();

        pros::delay(10);
    }
}
