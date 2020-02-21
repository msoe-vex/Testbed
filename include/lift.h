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

  pros::ADIDigitalIn *liftLimitSwitch;

  lift(int leftLiftMotorPort, int rightLiftMotorPort, int liftLimitPort);

  void manualControl(pros::Controller controller);

  double getPosition();

  double getVelocity();

  void setLiftState(liftState liftState);

  void setLiftState(liftState liftState, double setpoint);

  void periodic();

  ~lift();

private:
  void setPower(int liftPower);

  void setPIDPosition(double position);

  void setPIDVelocity(double velocity);
};
