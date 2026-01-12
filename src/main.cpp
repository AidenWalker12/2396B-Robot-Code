#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

void updateMotorStates();
void Penumatics();
extern int config;
extern int GLPConfig;
extern pros::adi::Pneumatics hood;
extern pros::adi::Pneumatics scraper;
extern lemlib::Chassis chassis;
Controller MasterController(pros::E_CONTROLLER_MASTER);
pros::adi::AnalogIn autosensor('E');

//! Initialize 
void initialize() {

}
void disabled() {
}
void competition_initialize() {
        pros::lcd::initialize();
    chassis.calibrate();

    // Motor Brake Modes
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    firststage.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    secondstage.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    pros::Task screenTask([&]() {
        //Odometry Data
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // Angle
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::lcd::print(3, "%f", firststage.get_actual_velocity());
            pros::lcd::print(4, "%f", secondstage.get_actual_velocity());
            updateMotorStates();
            pros::delay(50);
        }
    });
}
//! Autonomous
void autonomous() {

        float angle = (float)autosensor.get_value() * (270.0 / 4095.0); 
            if(angle >= 0 && angle  <= 30 ) {
            pros::lcd::print(3, "Left Red"); // Left Red Autonomous Routine

                scraper.set_value(false);
                config = UP;
                updateMotorStates();
                chassis.moveToPose(2.37, 30.5, -0.67, 2500, {.maxSpeed = 70});
                pros::delay(2500);
                config = NORUN;
                updateMotorStates();
                chassis.turnToHeading(77, 900,{.maxSpeed = 63});
                // chassis.moveToPoint(11.8, 36.1, 1000);
                // pros::delay(1000);
                // config = CENTERGOAL;
                // updateMotorStates();
                // pros::delay( 1000);
                // config = UP;
                // pros::delay( 400);
                // config = NORUN;
                // updateMotorStates();
                chassis.moveToPoint(-32.98, 11.55, 2000,{.forwards = false});
                updateMotorStates();
                chassis.turnToHeading(200,1200, {.maxSpeed = 63});
                scraper.set_value(true);
                pros::delay(250);
                config = UP;
                updateMotorStates();
                chassis.moveToPoint(-42, 3.47, 1500, {.maxSpeed = 40});
                chassis.moveToPoint(-35, 18.6, 1500, { .forwards = false});
                config = DOWN;
                updateMotorStates();
                pros::delay(250);
                config = NORUN;
                updateMotorStates();
                chassis.moveToPoint(-29.8, 26, 1000,{.forwards = false, .maxSpeed = 80 });
                chassis.moveToPoint(-28.6, 35, 2500, {.forwards = false, .minSpeed = 100});
                pros::delay (1000);
                hood.set_value(true);
                config = UP;
                updateMotorStates();

            }
            else if(angle > 30 && angle <= 125) {
            pros::lcd::print(3, "Left Blue"); // Left Blue Autonomous Routine


            }
            else if(angle > 125 && angle <= 240) {
            pros::lcd::print(3, "Right Blue"); // Right Blue Autonomous Routine


            }
            else if(angle > 240 && angle <= 270) {
            pros::lcd::print(3, "Right Red"); // Right Red Autonomous Routine
                scraper.set_value(false);
                config = UP;
                updateMotorStates();
                chassis.moveToPose(-0.37, 30.5, 0.67, 2500, {.maxSpeed = 70});
                pros::delay(2450);
                config = NORUN;
                updateMotorStates();
                chassis.turnToHeading(-77, 900,{.maxSpeed = 63});
                chassis.moveToPoint(-11.3, 35.9, 1000);
                pros::delay(1000);
                config = MODDOWN;
                updateMotorStates();
                pros::delay( 1700);
                config = UP;
                pros::delay( 400);
                config = NORUN;
                updateMotorStates();
                chassis.moveToPoint(32.98, 11.55, 2000,{.forwards = false});
                updateMotorStates();
                chassis.turnToHeading(-215,800, {.maxSpeed = 63});
                scraper.set_value(true);
                config = UP;
                updateMotorStates();
                chassis.moveToPoint(36.4, 5.4, 1300);
                chassis.moveToPoint(36.4, -12, 1500, {.maxSpeed = 45});
                config = UP;
                updateMotorStates();
                pros::delay(1000
                
                );
                config = DOWN;
                updateMotorStates();
                pros::delay(250);
                config = NORUN;
                updateMotorStates();
                chassis.moveToPoint(29, 23.4, 1000,{.forwards = false, .minSpeed = 70});
                chassis.moveToPoint(28.7, 36, 2500, {.forwards = false, .minSpeed = 100});
                hood.set_value(true);
                pros::delay (500);
                config = UP;
                updateMotorStates();

    }
}
//! Operator Control 
void opcontrol() {

    while (true) {

        // Arcade drive 
        int leftY  = MasterController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = MasterController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX, false, 0.75);

        // Config toggles 
        if (MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
                        config = (config == CENTERGOAL)  ? NORUN : CENTERGOAL;
        if (MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
                        config = (config == UP)          ? NORUN : UP;
        if (MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
                        config = (config == DOWN)        ? NORUN : DOWN;
        if (MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT) && MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_UP))
                        chassis.setPose(0,0,0);
        //Updates
        updateMotorStates();
        Penumatics();
        
        //Delay to save resources
        pros::delay(10);
    }
}