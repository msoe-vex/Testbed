/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * Copyright (c) 2017-2019, Purdue University ACM SIGBots.
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
 * not convienent for most student programmers.
 */
#define PROS_USE_SIMPLE_NAMES

/**
 * If defined, C++ literals will be available for use. All literals are in the
 * pros::literals namespace.
 *
 * For instance, you can do `4_mtr = 50` to set motor 4's target velocity to 50
 */
#define PROS_USE_LITERALS

#include "api.h"

/**
 * You should add more #includes here
 */
//#include "okapi/api.hpp"

#include "okapi/api/chassis/controller/chassisControllerIntegrated.hpp"
#include "okapi/api/chassis/controller/chassisControllerPid.hpp"
#include "okapi/api/chassis/controller/chassisScales.hpp"
#include "okapi/api/chassis/model/readOnlyChassisModel.hpp"
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "okapi/api/chassis/model/threeEncoderSkidSteerModel.hpp"
#include "okapi/api/chassis/model/xDriveModel.hpp"
#include "okapi/impl/chassis/controller/chassisControllerFactory.hpp"
#include "okapi/impl/chassis/model/chassisModelFactory.hpp"
//
// #include "okapi/api/control/async/asyncLinearMotionProfileController.hpp"
// #include "okapi/api/control/async/asyncMotionProfileController.hpp"
// #include "okapi/api/control/async/asyncPosIntegratedController.hpp"
// #include "okapi/api/control/async/asyncPosPidController.hpp"
// #include "okapi/api/control/async/asyncVelIntegratedController.hpp"
// #include "okapi/api/control/async/asyncVelPidController.hpp"
// #include "okapi/api/control/async/asyncWrapper.hpp"
// #include "okapi/api/control/controllerInput.hpp"
// #include "okapi/api/control/controllerOutput.hpp"
// #include "okapi/api/control/iterative/iterativeMotorVelocityController.hpp"
// #include "okapi/api/control/iterative/iterativePosPidController.hpp"
// #include "okapi/api/control/iterative/iterativeVelPidController.hpp"
// #include "okapi/api/control/util/controllerRunner.hpp"
// #include "okapi/api/control/util/flywheelSimulator.hpp"
// #include "okapi/api/control/util/pidTuner.hpp"
// #include "okapi/api/control/util/settledUtil.hpp"
// #include "okapi/impl/control/async/asyncControllerFactory.hpp"
// #include "okapi/impl/control/iterative/iterativeControllerFactory.hpp"
// #include "okapi/impl/control/util/controllerRunnerFactory.hpp"
// #include "okapi/impl/control/util/pidTunerFactory.hpp"
// #include "okapi/impl/control/util/settledUtilFactory.hpp"

#include "okapi/api/device/rotarysensor/continuousRotarySensor.hpp"
#include "okapi/api/device/rotarysensor/rotarySensor.hpp"
#include "okapi/impl/device/adiUltrasonic.hpp"
#include "okapi/impl/device/button/adiButton.hpp"
#include "okapi/impl/device/button/controllerButton.hpp"
#include "okapi/impl/device/controller.hpp"
#include "okapi/impl/device/motor/motor.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include "okapi/impl/device/rotarysensor/adiEncoder.hpp"
#include "okapi/impl/device/rotarysensor/adiGyro.hpp"
#include "okapi/impl/device/rotarysensor/integratedEncoder.hpp"
#include "okapi/impl/device/rotarysensor/potentiometer.hpp"
#include "okapi/impl/device/vision.hpp"

#include "okapi/api/filter/averageFilter.hpp"
#include "okapi/api/filter/composableFilter.hpp"
#include "okapi/api/filter/demaFilter.hpp"
#include "okapi/api/filter/ekfFilter.hpp"
#include "okapi/api/filter/emaFilter.hpp"
#include "okapi/api/filter/filter.hpp"
#include "okapi/api/filter/filteredControllerInput.hpp"
#include "okapi/api/filter/medianFilter.hpp"
#include "okapi/api/filter/passthroughFilter.hpp"
#include "okapi/api/filter/velMath.hpp"
#include "okapi/impl/filter/velMathFactory.hpp"

#include "okapi/api/units/QAcceleration.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QAngularAcceleration.hpp"
#include "okapi/api/units/QAngularJerk.hpp"
#include "okapi/api/units/QAngularSpeed.hpp"
#include "okapi/api/units/QArea.hpp"
#include "okapi/api/units/QForce.hpp"
#include "okapi/api/units/QFrequency.hpp"
#include "okapi/api/units/QJerk.hpp"
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/QMass.hpp"
#include "okapi/api/units/QPressure.hpp"
#include "okapi/api/units/QSpeed.hpp"
#include "okapi/api/units/QTime.hpp"
#include "okapi/api/units/QTorque.hpp"
#include "okapi/api/units/QVolume.hpp"

#include "okapi/api/util/abstractRate.hpp"
#include "okapi/api/util/abstractTimer.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include "okapi/api/util/supplier.hpp"
#include "okapi/api/util/timeUtil.hpp"
#include "okapi/impl/util/rate.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"
#include "okapi/impl/util/timer.hpp"

//#include "pros/api_legacy.h"

/**
 * If you find doing pros::Motor() to be tedious and you'd prefer just to do
 * Motor, you can use the namespace with the following commented out line.
 *
 * IMPORTANT: Only the okapi or pros namespace may be used, not both
 * concurrently! The okapi namespace will export all symbols inside the pros
 * namespace.
 */
// using namespace pros;
// using namespace pros::literals;
// using namespace okapi;

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
//#include <iostream>
#endif

#endif  // _PROS_MAIN_H_
