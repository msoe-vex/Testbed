#pragma once

#include "API.h"

namespace constants {
const int LEFT_FRONT_DRIVE_PORT = 19;
const int LEFT_REAR_DRIVE_PORT = 2;
const int RIGHT_FRONT_DRIVE_PORT = 20;
const int RIGHT_REAR_DRIVE_PORT = 1;
const int LEFT_LIFT_PORT = 9;
const int RIGHT_LIFT_PORT = 8;
const int LEFT_INTAKE_PORT = 6;
const int RIGHT_INTAKE_PORT = 5;
const int TRAY_TILT_PORT = 7;

const int LIFT_LIMIT_PORT = 0;

const double LOW_GOAL_POS = 2000.0;
const double FOUR_STACK_POS = 2400.0;
const double MID_GOAL_POS = 2600.0;
const double HIGH_GOAL_POS = 4280.0;

const double LIFT_MAX_VEL = 160.0;

const pros::motor_pid_s_t LIFT_POS_PID_VAL = pros::Motor::convert_pid(0, 0, 0, 0);
const pros::motor_pid_s_t LIFT_VEL_PID_VAL = pros::Motor::convert_pid(0, 0, 0, 0);
};
