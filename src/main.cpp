#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/rtos.hpp"
#include <sys/_intsup.h>

void updateMotorStates();
void Penumatics();
extern int config;
extern int GLPConfig;
extern pros::adi::Pneumatics hood;
extern pros::adi::Pneumatics scraper;
extern pros::adi::Pneumatics des;
extern pros::adi::Pneumatics fhood;
extern lemlib::Chassis chassis;
extern int INTAKE_SPEED;
extern int TOP_SPEED;
extern int FIRSTSTAGE_SPEED ;
extern int SECONDSTAGE_SPEED;
Controller MasterController(pros::E_CONTROLLER_MASTER);
pros::adi::AnalogIn autosensor('E');
    static float savedX = 0, savedY = 0, savedTheta = 0;

//! Initialize 
void initialize() {}

void disabled() {}

//! Competition Initialize 
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
            pros::lcd::print(3, "X: %f", savedX);
            pros::lcd::print(4, "Y: %f", savedY);
            pros::lcd::print(5, "Theta: %f", savedTheta);
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            updateMotorStates();     
            
            pros::delay(50);
            
        }
    });
}
//! Autonomous
void autonomous() {

    //! Tuning Test
    // chassis.moveToPoint(0, 48, 5000);
    // chassis.turnToHeading(-90, 5000);
    // chassis.moveToPoint(-48, 48, 5000);
    // chassis.turnToHeading(-180, 5000);
    // chassis.moveToPoint(-48, 0, 5000);
    // chassis.turnToHeading(-270, 5000);
    // chassis.moveToPoint(0, 0, 5000);
    // chassis.turnToHeading(0, 5000);
    // chassis.moveToPoint(0, 0, 5000);
    //! Four Block Rush Right
    // chassis.setPose(0,0, 20.31);
    // config = UP;
    // updateMotorStates();
    // chassis.moveToPoint(4.42, 11.76,1500 );
    // chassis.moveToPoint(10.15,  25,1500, {}, false );
    // pros::delay(250);
    // scraper.set_value(true);
    // pros::delay(1500);
    // chassis.moveToPoint(11.2, 28.52, 1500);
    // chassis.turnToHeading(-40.65, 1500);
    // pros::delay(1000);
    // scraper.set_value(false);
    // config = NORUN;
    // updateMotorStates();
    // INTAKE_SPEED = 100;
    // TOP_SPEED    = 100;
    // FIRSTSTAGE_SPEED   = 100;
    // SECONDSTAGE_SPEED  = 100;
    // chassis.moveToPoint(2.94, 33.1, 1000);
    // chassis.turnToHeading(-47.2, 1000);
    // chassis.moveToPoint(-3.7, 36.4,1500  );
    // chassis.turnToHeading(-44.6, 1500);
    // pros::delay(1500);
    // config = MODDOWN;
    // updateMotorStates();
    // pros::delay(1500);
    // config = DOWN;
    // updateMotorStates();



        // float angle = (float)autosensor.get_value() * (270.0 / 4095.0); 
        //     if(angle >= 0 && angle  <= 30 ) {
        //     pros::lcd::print(3, "Left Red"); // Left Red Autonomous Routine

                // scraper.set_value(false);
                // config = UP;
                // updateMotorStates();
                // chassis.moveToPose(1.3, 27.7, 7.3, 2500, {.maxSpeed = 70});
                // pros::delay(2500);
                // config = NORUN;
                // updateMotorStates();
                // chassis.turnToHeading(69, 900,{.maxSpeed = 63});
                // chassis.moveToPoint(10.3, 28.4, 1000);
                // pros::delay(1000);
                // config = CENTERGOAL;
                // updateMotorStates();
                // pros::delay( 1000);
                // config = UP;
                // pros::delay( 400);
                // config = NORUN;
                // updateMotorStates();
                // chassis.moveToPoint(-32.98, 11.55, 2000,{.forwards = false});
                // updateMotorStates();
                // chassis.turnToHeading(200,1200, {.maxSpeed = 63});
                // scraper.set_value(true);
                // pros::delay(250);
                // config = UP;
                // updateMotorStates();
                // chassis.moveToPoint(-42, 3.47, 1500, {.maxSpeed = 40});
                // chassis.moveToPoint(-35, 18.6, 1500, { .forwards = false});
                // config = DOWN;
                // updateMotorStates();
                // pros::delay(250);
                // config = NORUN;
                // updateMotorStates();
                // chassis.moveToPoint(-29.8, 26, 1000,{.forwards = false, .maxSpeed = 80 });
                // chassis.moveToPoint(-28.6, 35, 2500, {.forwards = false, .minSpeed = 100});
                // pros::delay (1000);
                // hood.set_value(true);
                // config = UP;
                // updateMotorStates();

            // }
            // else if(angle > 30 && angle <= 125) {
            // pros::lcd::print(3, "Left Blue"); // Left Blue Autonomous Routine


            // }
            // else if(angle > 125 && angle <= 240) {
            // pros::lcd::print(3, "Right Blue"); // Right Blue Autonomous Routine


            // }
            // else if(angle > 240 && angle <= 270) {
            // pros::lcd::print(3, "Right Red"); // Right Red Autonomous Routine
                // scraper.set_value(false);
                // config = UP;
                // updateMotorStates();
                // chassis.moveToPoint(0.5, 15.50, 1000);
                // chassis.moveToPoint(1, 31.00, 1200, {.maxSpeed = 37});
                // pros::delay(2200);
                // config = NORUN;
                // updateMotorStates();
                // chassis.turnToHeading(-75, 900,{.maxSpeed = 63});
                // chassis.moveToPoint(-17.27, 28.05, 800);
                // chassis.turnToHeading(-72, 500);
                // pros::delay(100);
                // config = MODDOWN;
                // updateMotorStates();
                // pros::delay( 1300);
                // config = UP;
                // pros::delay( 400);
                // config = NORUN;
                // updateMotorStates();
                // chassis.moveToPoint(26.40, 10.83, 2000,{.forwards = false});
                // updateMotorStates();
                // chassis.turnToHeading(-220,1000, {.maxSpeed = 63});
                // scraper.set_value(true);
                // config = UP;
                // updateMotorStates();
                // chassis.moveToPoint(41.08, -7.47, 1300, {.minSpeed = 63});
                // config = UP;
                // updateMotorStates();
                // pros::delay(2000);
                // config = DOWN;
                // updateMotorStates();
                // pros::delay(250);
                // config = NORUN;
                // updateMotorStates();
                // chassis.moveToPose(34, 8.5, -200, 1000, {.forwards = false});
                // chassis.moveToPoint(29.5, 19.9, 3000, {.forwards = false, .minSpeed = 60});
                // hood.set_value(true);
                // pros::delay (1750);
                // config = UP;
                // updateMotorStates();

    // }


    //! Skillz
    chassis.moveToPoint(0, 7, 2000);
    chassis.turnToHeading(90, 2000);
    chassis.moveToPoint(48, 1, 3000, {.maxSpeed = 63});
    chassis.turnToHeading(180, 2000);
    pros::delay(2000);
    chassis.setPose(48,5,180);
    //Turn to face the loader
    scraper.set_value(true);
    config = UP;
    chassis.moveToPoint(45.5, -20, 2000, {.maxSpeed = 63});
    chassis.moveToPoint(45.5, -22, 1000);
    pros::delay(1500);
    config = NORUN;
    chassis.moveToPoint(36, 16, 2000, {.forwards = false});
    chassis.moveToPose(36, 102, 180, 2000, {.forwards = false});
    chassis.swingToHeading(0,  DriveSide::LEFT, 2000);
    chassis.moveToPoint(44.6, 67, 1000, {.forwards = false});

    
}
//! Operator Control 
void opcontrol() {

    //Set Initial Speeds for Driver Control
    INTAKE_SPEED       = 600;
    TOP_SPEED          = 600;
    FIRSTSTAGE_SPEED   = 600;
    SECONDSTAGE_SPEED  = 600;

    //Repeat Function for Driver Control
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

        // To save odometry points
        if (MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            savedX = chassis.getPose().x;
            savedY = chassis.getPose().y;
            savedTheta = chassis.getPose().theta;
        }

        //Delay to save resources
        pros::delay(10);
    }
}