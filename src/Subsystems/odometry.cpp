#include "/Subsystems/odometry.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "Subsystems/motors.hpp"
//#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"

// Sensors
pros::Imu imu(16);
pros::Rotation horizontalEnc(17);
pros::Rotation verticalEnc(18);

lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -6.7);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -0.5);

lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 10.896752,
                              lemlib::Omniwheel::NEW_275, 600, 6);

lemlib::ControllerSettings linearController(10,0,3,0,0,0,0,0,0);
lemlib::ControllerSettings angularController(2,0,10,0,0,0,0,0,0);

lemlib::OdomSensors sensors(&vertical, nullptr, &horizontal, nullptr, &imu);

lemlib::ExpoDriveCurve throttleCurve(3,10,1.019);
lemlib::ExpoDriveCurve steerCurve(3,10,1.019);

lemlib::Chassis chassis(drivetrain, linearController, angularController,
                        sensors, &throttleCurve, &steerCurve);

void initOdometry() {
    chassis.calibrate();
}
