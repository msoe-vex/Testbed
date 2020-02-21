#pragma once

#include "API.h"

namespace constants {
const double LOW_GOAL_POS = 2000.0;
const double FOUR_STACK_POS = 2400.0;
const double MID_GOAL_POS = 2600.0;
const double HIGH_GOAL_POS = 4280.0;

const double LIFT_MAX_VEL = 160.0;

const pros::motor_pid_s_t LIFT_POS_PID_VAL = pros::Motor::convert_pid(0, 0, 0, 0);
const pros::motor_pid_s_t LIFT_VEL_PID_VAL = pros::Motor::convert_pid(0, 0, 0, 0);
};
