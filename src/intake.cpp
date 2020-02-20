#include "intake.h"

intake::intake(int leftIntakeMotorPort, int rightIntakeMotorPort, int intakePivotMotorPort) {
  leftIntakeMotor = new pros::Motor(leftIntakeMotorPort, true);
  rightIntakeMotor = new pros::Motor(rightIntakeMotorPort, false);

  intakePivotMotor = new pros::Motor(intakePivotMotorPort, false);
}

void intake::setSpeed(int leftIntakeSpeed, int rightIntakeSpeed) {
  leftIntakeMotor->move(leftIntakeSpeed);
  rightIntakeMotor->move(rightIntakeSpeed);
}

void intake::pivotUp() {

}

void intake::pivotDown() {

}

void intake::pivot(int speed) {
  intakePivotMotor->move(speed);
}

intake::~intake() {
  free(leftIntakeMotor);
  free(rightIntakeMotor);
  free(intakePivotMotor);

  leftIntakeMotor = 0;
  rightIntakeMotor = 0;
  intakePivotMotor = 0;
}
