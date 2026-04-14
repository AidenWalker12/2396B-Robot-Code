/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * \copyright Copyright (c) 2017-2024, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

/**
 * If defined, some commonly used enums will have preprocessor macros which give
 * a shorter, more convenient naming pattern. If this isn't desired, simply
 * comment the following line out.
 *
 * For instance, E_CONTROLLER_MASTER has a shorter name: CONTROLLER_MASTER.
 * E_CONTROLLER_MASTER is pedantically correct within the PROS styleguide, but
 * not convenient for most student programmers.
 */
#define PROS_USE_SIMPLE_NAMES

/**
 * If defined, C++ literals will be available for use. All literals are in the
 * pros::literals namespace.
 *
 * For instance, you can do `4_mtr = 50` to set motor 4's target velocity to 50
 */
#define PROS_USE_LITERALS

#include "api.h" // IWYU pragma: keep
// Add lemlib declarations for shared globals used across source files
#include "lemlib/api.hpp" // IWYU pragma: keep

/**
 * You should add more #includes here
 */


/**
 * If you find doing pros::Motor() to be tedious and you'd prefer just to do
 * Motor, you can use the namespace with the following commented out line.
 *
 * IMPORTANT: Only the okapi or pros namespace may be used, not both
 * concurrently! The okapi namespace will export all symbols inside the pros
 * namespace.
 */
using namespace pros;

/**
 * Prototypes for the competition control tasks are redefined here to ensure
 * that they can be called from user code (i.e. calling autonomous from a
 * button press in opcontrol() for testing purposes).
 */
#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */
#include <iostream> // IWYU pragma: keep
#endif  // __cplusplus

// Shared globals and types used across multiple translation units
// Motor / sensor types are available via `api.h` and `lemlib/api.hpp` above
// Motor state enums
enum State { OFF = 0, FORWARD = 1, REVERSE = 2, STOMPUP = 3, STOMPDOWN = 4, STOMPCENTER = 5 };
enum Config { NORUN = 0, UP = 1, DOWN = 2, CENTERGOAL = 3, MODDOWN = 4};

// Forward declarations for shared globals (defined in src/main.cpp)
extern Controller MasterController;
extern MotorGroup leftMotors;
extern MotorGroup rightMotors;
extern Motor intake;
extern Motor firststage;
extern Motor secondstage;


extern Rotation horizontalEnc;
extern Rotation verticalEnc;

extern int config;
extern int effectiveConfig;

// Shared function prototypes
void updateMotorStates();

#endif  // _PROS_MAIN_H_
