#pragma once

#include "API.h"

class intake {
public:
  pros::Motor *leftIntakeMotor, *rightIntakeMotor, *intakePivotMotor;

  intake(int leftIntakeMotorPort, int rightIntakeMotorPort, int intakePivotMotorPort);

  void setSpeed(int leftSpeed, int rightSpeed);

  void pivotUp();

  void pivotDown();

  ~intake();

  void pivot(int speed);
};
