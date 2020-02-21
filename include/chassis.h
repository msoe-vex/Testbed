#pragma once

#include "API.h"

class chassis {
public:
  pros::Motor *leftFrontDriveMotor, *leftRearDriveMotor,
              *rightFrontDriveMotor, *rightRearDriveMotor;

  chassis(int leftFrontDriveMotorPort, int leftRearDriveMotorPort,
          int rightFrontDriveMotorPort, int rightRearDriveMotorPort);

  void setSpeed(int leftDriveSpeed, int rightDriveSpeed);

  double getLeftSpeed();

  double getRightSpeed();

  void periodic();

  ~chassis();
};
