#pragma once

#include "API.h"
#include "Constants.h"

class Intake {
public:
  enum class intakeState {
    Manual,
    Hold
  };

  intakeState currentIntakeState;

  pros::Motor *leftIntakeMotor, *rightIntakeMotor;

  Intake(int leftIntakeMotorPort, int rightIntakeMotorPort);

  void SetSpeed(int leftSpeed, int rightSpeed);

  void HoldIntake();

  void Periodic();

  void ManualControl(pros::Controller controller);

  ~Intake();

private:
  void SetPIDPositionRelative(double position);
};
