#pragma once

#include "api.h"
#include "Chassis.h"
#include "Lift.h"
#include "Intake.h"
#include "Tilt.h"

class Robot {
public:
  Robot();

  Chassis chassis;

  Lift lift;

  Intake intake;

  Tilt tilt;
};
