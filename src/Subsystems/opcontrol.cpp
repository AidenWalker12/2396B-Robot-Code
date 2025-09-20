#include "Subsystems/motors.hpp"
#include "Subsystems/odometry.hpp"
#include "Subsystems/config.hpp"
#include "pros/rtos.hpp"

void opcontrol() {
    while (true) {
        // Arcade drive
        int leftY = Controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = Controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX, false, 0.75);

        // Config toggles
        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) config = (config == UP) ? NORUN : UP;
        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) config = (config == DOWN) ? NORUN : DOWN;
        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) config = (config == STORAGEIN) ? NORUN : STORAGEIN;
        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) config = (config == STORAGELONG) ? NORUN : STORAGELONG;

        // Update motors
        updateMotorStates();

        pros::delay(10);
    }
}
