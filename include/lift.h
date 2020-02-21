#pragma once

#include "API.h"
#include "Constants.h"

class lift {
public:
  enum class liftState {
    Manual,
    LowGoal,
    FourStack,
    MediumGoal,
    HighGoal,
    VariableSetpointPID
  };

  liftState currentLiftState;

  bool isStateScoring = false;

  double currentSetpoint = 0;

  pros::Motor *leftLiftMotor, *rightLiftMotor;

  lift(int leftLiftMotorPort, int rightLiftMotorPort);

  double getPosition();

  double getVelocity();

  void setLiftState(pros::Controller controller);

  void setLiftState(liftState liftState);

  void setLiftState(liftState liftState, double setpoint);

  void periodic(pros::Controller controller);

  ~lift();

private:
  void manualControl(pros::Controller controller);

  void setPower(int liftPower);

  void setPIDPosition(double position);

  void setPIDVelocity(double velocity);
};
