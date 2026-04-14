#include "main.h"
#include "lemlib/chassis/chassis.hpp"

void updateMotorStates();
void Penumatics();
void AutoScore(bool resetpos, bool resetimu, int x, int y);
void AutoReset();
extern int config, GLPConfig, FIRSTSTAGE_SPEED, SECONDSTAGE_SPEED;
extern adi::Pneumatics hood, scraper, des, fhood;
extern lemlib::Chassis chassis;
extern Imu imu;
extern State firststageState, secondstageState, STOMP;
Controller MasterController(pros::E_CONTROLLER_MASTER);
adi::AnalogIn autosensor('E');
static float savedX = 0, savedY = 0, savedTheta = 0;
float thetaauto = 0;
extern int autoreset;
extern float YSkills, XSkills;

//! Initialize 
void initialize() {}

void disabled() {}


//! Competition Initialize 
void competition_initialize() {
    lcd::initialize();
    chassis.calibrate();

    delay(750);

    // Motor Brake Modes
    firststage  .set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    secondstage .set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    leftMotors  .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors .set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    pros::Task screenTask([&]() {
        while (true) {

            // print robot location to the brain screen
            lcd::print(0, "X: %f", chassis.getPose().x);         // x
            lcd::print(1, "Y: %f", chassis.getPose().y);         // y
            lcd::print(2, "Theta: %f", chassis.getPose().theta); // Angle
            lcd::print(3, "FirstStage Temp: %f", firststage.get_temperature());
            lcd::print(4, "Secondstage Temp: %f", secondstage.get_temperature());

            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            updateMotorStates();  

        XSkills = chassis.getPose().x;  
        YSkills = chassis.getPose().y;
        AutoReset();
        chassis.setPose(XSkills, YSkills, (imu.get_heading()));   

            
            delay(50);
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

    //! Tuning Test 2
        // chassis.turnToHeading(90, 99999999);
    
    // //! Four Block Rush Right
        // chassis.setPose(0,0, 20.31);
        // config = UP;
        // updateMotorStates();
        // chassis.moveToPoint(4.42, 10.76,1500 );
        // chassis.moveToPoint(10.15,  25,1500, {}, true );
        // pros::delay(1000);
        // scraper.set_value(true);
        // pros::delay(1500);
        // chassis.moveToPoint(11.2, 27.52, 1500);
        // chassis.turnToHeading(-40.65, 1500);
        // pros::delay(1000);
        // scraper.set_value(false);
        // config = NORUN;
        // updateMotorStates();
        // FIRSTSTAGE_SPEED   = 360;
        // SECONDSTAGE_SPEED  = 450;
        // chassis.moveToPoint(2.94, 33.1, 1000);
        // chassis.turnToHeading(-47.2, 1000);
        // chassis.moveToPoint(-2.7, 36.4,1500  );
        // chassis.turnToHeading(-44.6, 1500);
        // pros::delay(1500);
        // config = MODDOWN;
        // updateMotorStates();
        // pros::delay(1500);
        // config = DOWN;
        // updateMotorStates();

    //! Dany Auton
        // chassis.moveToPoint(0, 3, 1500);

    //! Skillz
        chassis.moveToPoint(0, 26, 1500, {.minSpeed = 20,.earlyExitRange = 6});
        // chassis.moveToPose(12,18, 90, 1500, {.lead = 1,.earlyExitRange = 2});
        chassis.moveToPose(31, 5, 180, 2500, { .lead = .4,.minSpeed = 10, .earlyExitRange = 1}, true);
        scraper.extend();
        SECONDSTAGE_SPEED = 300;
        config = UP;
        chassis.moveToPoint(32, -5, 3000, {.minSpeed = 25, .earlyExitRange = 0.000}, false);
        config = NORUN;
        // Start move to the long goal
        chassis.moveToPoint(29, 7, 3000, {.forwards = false,  .minSpeed = 35, .earlyExitRange = 2}, false);
        chassis.moveToPose(47, 32, 180, 5000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 2});
        chassis.moveToPose(48, 101, 181, 6000, {.forwards = false, .minSpeed = 80, .earlyExitRange = 2});
        chassis.moveToPose(32, 90, 0, 2000, {.forwards = false});
        // chassis.swingToHeading(345, DriveSide::RIGHT, 2000,{ .direction = AngularDirection::CCW_COUNTERCLOCKWISE});
        // chassis.moveToPose(42, 83, 1, 2000, {.forwards = false, .maxSpeed = 40, .earlyExitRange = .5});
        // Slam into long goal and than score
        chassis.moveToPose(31, 90, 0, 1500,{.forwards = false}, false);
        chassis.moveToPoint(31, 40, 1500,{.forwards = false, .minSpeed = 60}, false);
        delay(300);
        AutoScore(true, true, 0, 0);
        // Begin move to the second loader
        chassis.moveToPoint(0, 24, 1500, {.minSpeed = 20, .earlyExitRange = 2}, false);
        hood.retract();
        SECONDSTAGE_SPEED = 300;
        chassis.moveToPoint(1, 33, 3000, {.minSpeed = 27, .earlyExitRange = 0.000}, false);
        chassis.moveToPoint(1, 17.5, 1500,{.forwards = false, .minSpeed = 50, .earlyExitRange = 2}, false);
        // Slam into the long goal and score again
        chassis.moveToPose(0, 0, 0, 1500,{.forwards = false, .minSpeed = 80}, false);
        chassis.moveToPose(0, -40, 0, 1500,{.forwards = false, .maxSpeed = 80}, false);
        delay(500);
        AutoScore(true, true, 0, 0);
    //! Cont
        chassis.moveToPoint(0, 6, 1000, {.minSpeed = 50, .earlyExitRange = 1});
        chassis.moveToPose(-80, 10, 270, 7000, {.lead = .3, .minSpeed = 80, .earlyExitRange = 2});
        chassis.moveToPose(-96, 23, 0, 3000,{}, false);
        hood.retract();
        SECONDSTAGE_SPEED = 300;
        config = UP;
        chassis.moveToPoint(-95, 35, 3000, {.minSpeed = 27, .earlyExitRange = 0.000}, false);
        // pose (-96, 33, 0) is the second loader
        config = NORUN;
        // Start move to the long goal
        chassis.moveToPoint(-94, 21, 3000, {.forwards = false,  .minSpeed = 35, .earlyExitRange = 2}, false);
        chassis.moveToPose(-112, -4, 360, 5000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 2});
        chassis.moveToPose(-113, -73, 1, 6000, {.forwards = false, .minSpeed = 80, .earlyExitRange = 2});
        chassis.moveToPose(-98, -60, 180, 1500,{.forwards = false}, false);
        // chassis.swingToHeading(165, DriveSide::RIGHT, 2000,{ .direction = AngularDirection::CCW_COUNTERCLOCKWISE});
        chassis.moveToPose(-98, -53.5, 180, 2000, {.forwards = false, .maxSpeed = 40, .earlyExitRange = .5});
        // Slam into long goal and than score
        chassis.moveToPoint(-98, -12, 1000,{.forwards = false, .minSpeed = 80}, false);
        delay(300);
        AutoScore(true, true, 0, 0);
        // Begin move to the second loader
        chassis.moveToPoint(0, 24, 1500, {.minSpeed = 20, .earlyExitRange = 2}, false);
        hood.retract();
        SECONDSTAGE_SPEED = 300;
        chassis.moveToPoint(0, 35, 3000, {.minSpeed = 22, .earlyExitRange = 0.000}, false);
        // Slam into the long goal and score again
        chassis.moveToPoint(0, 17.5, 1500,{.forwards = false, .minSpeed = 50, .earlyExitRange = 2}, false);
        chassis.moveToPose(0, -40, 180, 1500,{.forwards = false, .minSpeed = 80}, false);
        delay(500);
        AutoScore(true, false, 0, 0);
        config = NORUN;
        scraper.retract();
        chassis.moveToPoint(0, 10, 500);
        chassis.moveToPoint(-50, 35, 3000, {.forwards = false});
}
//! Operator Control 
void opcontrol() {

    //Set Initial Speeds for Driver Control
    FIRSTSTAGE_SPEED   = 600;
    SECONDSTAGE_SPEED  = 600;

    //Repeat Function for Driver Control
    while (true) {

        // Arcade drive 
        int leftY  = MasterController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = MasterController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX, false, 0.75);

        // Controller
        if (MasterController.get_digital_new_press(E_CONTROLLER_DIGITAL_R1)) {
                        config = (config == CENTERGOAL)  ? NORUN : CENTERGOAL;
                FIRSTSTAGE_SPEED = 600;
                SECONDSTAGE_SPEED = 600;
        }
        if (MasterController.get_digital_new_press(E_CONTROLLER_DIGITAL_R2)) {
            config = (config == MODDOWN)  ? NORUN : MODDOWN;
            FIRSTSTAGE_SPEED =  400;
            SECONDSTAGE_SPEED = 400;
                        }
        if (MasterController.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)) {
                config = (config == UP)          ? NORUN : UP;
                FIRSTSTAGE_SPEED = 600;
                SECONDSTAGE_SPEED = 600;
        }
        if (MasterController.get_digital_new_press(E_CONTROLLER_DIGITAL_L2)) {
                config = (config == DOWN)        ? NORUN : DOWN;
                FIRSTSTAGE_SPEED = 600;
                SECONDSTAGE_SPEED = 600;
        }
        if (MasterController.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT) && MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_UP))
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