#pragma once

#include "API.h"
#include "Constants.h"

class Lift {
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

  Lift(int leftLiftMotorPort, int rightLiftMotorPort, int liftLimitPort);

  void ManualControl(pros::Controller controller);

  double GetPosition();

  double GetVelocity();

  void SetLiftState(liftState liftState);

  void SetLiftState(liftState liftState, double setpoint);

  bool GetAtDestination();

  void Periodic();

  void SetPIDPosition(double position);

  void SetPIDPosition(double position, double maxVel);

  ~Lift();

private:
  void SetPower(int liftPower);

  void SetPIDVelocity(double velocity);
};
