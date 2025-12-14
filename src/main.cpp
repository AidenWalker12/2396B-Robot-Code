#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

void updateMotorStates();
void Autosensor();
void Penumatics();
extern int config;
extern int GLPConfig;
extern float  angle;
extern pros::adi::Pneumatics hood;
extern pros::adi::Pneumatics scraper;
extern lemlib::Chassis chassis;
Controller MasterController(pros::E_CONTROLLER_MASTER);

//! Initialize 
void initialize() {
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
            pros::lcd::print(0, "Y: %f", chassis.getPose().y);         // y
            pros::lcd::print(0, "Theta: %f", chassis.getPose().theta); // Angle
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}


void disabled() {}
void competition_initialize() {}

//! Autonomous
void autonomous() {
    Autosensor();
        if(angle >= 0 && angle  <= 30 ) {
            pros::lcd::print(3, "Left Red"); // Left Red Autonomous Routine
           void RedLeft();
        }
        else if(angle > 30 && angle <= 125) {
            pros::lcd::print(3, "Left Blue"); // Left Blue Autonomous Routine
            void BlueLeft();
        }
        else if(angle > 125 && angle <= 240) {
            pros::lcd::print(3, "Right Blue"); // Right Blue Autonomous Routine
            void BlueRight();
        }
        else if(angle > 240 && angle <= 270) {
            pros::lcd::print(3, "Right Red"); // Right Red Autonomous Routine
            void RedRight();
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
 
        //Updates
        updateMotorStates();
        Penumatics();
        
        //Delay to save resources
        pros::delay(10);
    }
}