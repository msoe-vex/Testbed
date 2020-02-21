#pragma once

#include "api.h"
#include "Constants.h"

class Tilt {
public:
  pros::Motor *trayTiltMotor;

  pros::ADIDigitalIn *trayLimitSwitch;

  Tilt(int intakePivotMotorPort, int trayLimitPort);

  void PivotUp();

  void PivotDown();

  void ManualControl(pros::Controller controller);

  void Periodic();

  ~Tilt();

private:
  void Pivot(int speed);
};
