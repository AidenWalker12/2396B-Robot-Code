//#include "Subsystems/motors.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "Subsystems/odometry.hpp"

void autonomous() {
    // Example autonomous
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(24, 0, 2000); // forward 24"
    chassis.moveToPoint(0, 0, 2000);  // return back
}
