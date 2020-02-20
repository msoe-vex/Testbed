#pragma once

#include "API.h"

class lift {
public:
  pros::Motor *leftLiftMotor, *rightLiftMotor;

  lift(int leftLiftMotorPort, int rightLiftMotorPort);

  void setSpeed(int liftSpeed);

  void setPosition(int position);

  double getPosition();

  ~lift();
};
