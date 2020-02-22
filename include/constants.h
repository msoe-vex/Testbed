#pragma once

#include "API.h"

namespace constants {
const int LEFT_FRONT_DRIVE_PORT = 4;
const int LEFT_REAR_DRIVE_PORT = 2;
const int RIGHT_FRONT_DRIVE_PORT = 5;
const int RIGHT_REAR_DRIVE_PORT = 3;
const int LEFT_LIFT_PORT = 6;
const int LEFT_INTAKE_PORT = 8;
const int RIGHT_INTAKE_PORT = 9;
const int TRAY_TILT_PORT = 7;

const int LIFT_LIMIT_PORT = 1;
const int TRAY_LIMIT_PORT = 2;

const double LOW_GOAL_POS = 200.0;
const double MID_GOAL_POS = 400.0;

const double TRAY_SCORING_POS = 3650.0;

const double LIFT_MAX_VEL = 100.0;
const double TRAY_MAX_VEL = 200.0;

const pros::motor_pid_s_t LIFT_POS_PID_VAL = pros::Motor::convert_pid(0, 0, 0, 0);
const pros::motor_pid_s_t LIFT_VEL_PID_VAL = pros::Motor::convert_pid(0, 0, 0, 0);
};
