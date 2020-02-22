#pragma once

#include "API.h"

class Chassis {
public:
  pros::Motor *leftFrontDriveMotor, *leftRearDriveMotor,
              *rightFrontDriveMotor, *rightRearDriveMotor;

  Chassis(int leftFrontDriveMotorPort, int leftRearDriveMotorPort,
          int rightFrontDriveMotorPort, int rightRearDriveMotorPort);

  void SetSpeed(int leftDriveSpeed, int rightDriveSpeed);

  double GetLeftSpeed();

  double GetRightSpeed();

  void Periodic();

  void SetPIDPosition(double position);

  void SetPIDPosition(double position, double maxVel);

  ~Chassis();
};
