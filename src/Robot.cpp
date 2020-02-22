#include "Robot.h"

Robot::Robot() : chassis(constants::LEFT_FRONT_DRIVE_PORT, constants::LEFT_REAR_DRIVE_PORT,
  		            constants::RIGHT_FRONT_DRIVE_PORT, constants::RIGHT_REAR_DRIVE_PORT),
                 lift(constants::LEFT_LIFT_PORT, constants::RIGHT_LIFT_PORT,
                  constants::LIFT_LIMIT_PORT),
                 intake(constants::LEFT_INTAKE_PORT, constants::RIGHT_INTAKE_PORT),
                 tilt(constants::TRAY_TILT_PORT, constants::TRAY_LIMIT_PORT) {

}
