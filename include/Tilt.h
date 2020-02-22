#pragma once

#include "api.h"
#include "Constants.h"

class Tilt {
public:
  enum class trayState {
    Manual,
    Down,
    Up
  };

  trayState currentTrayState;

  pros::Motor *trayTiltMotor;

  pros::ADIDigitalIn *trayLimitSwitch;

  Tilt(int intakePivotMotorPort, int trayLimitPort);

  void PivotUp();

  void PivotDown();

  void ManualControl(pros::Controller controller);

  int IsTrayDown();

  double GetTrayPosition();

  void Periodic();

  ~Tilt();

private:
  void Pivot(int speed);
};
