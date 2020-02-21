#include "Intake.h"

Intake::Intake(int leftIntakeMotorPort, int rightIntakeMotorPort) {
  leftIntakeMotor = new pros::Motor(leftIntakeMotorPort, true);
  rightIntakeMotor = new pros::Motor(rightIntakeMotorPort, false);
}

void Intake::SetSpeed(int leftIntakeSpeed, int rightIntakeSpeed) {
  leftIntakeMotor->move(leftIntakeSpeed);
  rightIntakeMotor->move(rightIntakeSpeed);
}

void Intake::ManualControl(pros::Controller controller) {
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) == 1) {
    SetSpeed(127, 127);
  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) == 1) {
    SetSpeed(-127, -127);
  } else {
    SetSpeed(0, 0);
  }
}

void Intake::Periodic() {

}

Intake::~Intake() {
  free(leftIntakeMotor);
  free(rightIntakeMotor);

  leftIntakeMotor = 0;
  rightIntakeMotor = 0;
}
