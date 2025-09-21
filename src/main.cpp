#include "main.h"
#include "control.cpp"
#include "subsystems.hpp" // IWYU pragma: keep
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "autonomous/main.auto.cpp"


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
void competition_initialize() {
        while (true) {
        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            autonmode = BLUERIGHT;
        }
        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
            autonmode = REDRIGHT;
        }
        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            autonmode = BLUELEFT;
        }
        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            autonmode = BLUELEFT;
        }
        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            autonmode = SKILLS;
        }

        // Display the current auton selection
        pros::lcd::print(0, "Auton Selected: %d", autonmode + 1);

        pros::delay(20);
    }
}

void autonomous() {
    switch(autonmode) {
        case BLUERIGHT:
            autonblueright();
            break;
        case BLUELEFT:
            autonblueleft();
            break;
        case REDRIGHT:
            autonredright();
            break;
        case REDLEFT:
            autonredleft();
            break;
        case SKILLS:
            autonskills();
            break;
    }
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
