#include "chassis.h"

chassis::chassis(int leftFrontDriveMotorPort, int leftRearDriveMotorPort,
                 int rightFrontDriveMotorPort, int rightRearDriveMotorPort) {
  leftFrontDriveMotor = new pros::Motor(leftFrontDriveMotorPort, false);
  leftRearDriveMotor = new pros::Motor(leftRearDriveMotorPort, false);
  rightFrontDriveMotor = new pros::Motor(rightFrontDriveMotorPort, true);
  rightRearDriveMotor = new pros::Motor(rightRearDriveMotorPort, true);
}

void chassis::setSpeed(int leftDriveSpeed, int rightDriveSpeed) {
  leftFrontDriveMotor->move(leftDriveSpeed);
  leftRearDriveMotor->move(leftDriveSpeed);
  rightFrontDriveMotor->move(rightDriveSpeed);
  rightRearDriveMotor->move(rightDriveSpeed);
}

chassis::~chassis() { // Deconstructor
  free(leftFrontDriveMotor); // Free memory
  free(leftRearDriveMotor); // Free memory
  free(rightFrontDriveMotor); // Free memory
  free(rightRearDriveMotor); // Free memory

  leftFrontDriveMotor = 0; // Clear reference
  leftRearDriveMotor = 0; // Clear reference
  rightFrontDriveMotor = 0; // Clear reference
  rightRearDriveMotor = 0;
}
