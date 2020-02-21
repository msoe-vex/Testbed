#pragma once

#include "API.h"
#include "Constants.h"

class Intake {
public:
  pros::Motor *leftIntakeMotor, *rightIntakeMotor;

  Intake(int leftIntakeMotorPort, int rightIntakeMotorPort);

  void SetSpeed(int leftSpeed, int rightSpeed);

  void HoldIntake();

  void Periodic();

  void ManualControl(pros::Controller controller);

  ~Intake();
};
