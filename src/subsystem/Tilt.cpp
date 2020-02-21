#include "Tilt.h"

Tilt::Tilt(int intakePivotMotorPort, int trayLimitPort) {
  trayTiltMotor = new pros::Motor(intakePivotMotorPort, false);

  trayLimitSwitch = new pros::ADIDigitalIn(trayLimitPort);
}

void Tilt::PivotUp() {
  trayTiltMotor->move_absolute(constants::TRAY_SCORING_POS, constants::TRAY_MAX_VEL);
}

void Tilt::PivotDown() {
  if (trayLimitSwitch->get_value() != 0) {
    trayTiltMotor->move(-127);
  } else {
    trayTiltMotor->move(0);
  }
}

void Tilt::ManualControl(pros::Controller controller) {
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) == 1) {
    Pivot(127);
  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B) == 1) {
    Pivot(-127);
  } else {
    Pivot(0);
  }
}

void Tilt::Periodic() {

}

Tilt::~Tilt() {

}

void Tilt::Pivot(int speed) {
  trayTiltMotor->move(speed);
}
