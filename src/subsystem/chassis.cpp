#include "Chassis.h"

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

double chassis::getLeftSpeed() {
  return (leftFrontDriveMotor->get_position() + leftRearDriveMotor->get_position()) / 2;
}

double chassis::getRightSpeed() {
  return (rightFrontDriveMotor->get_position() + rightRearDriveMotor->get_position()) / 2;
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
