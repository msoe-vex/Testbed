#include "Tilt.h"

Tilt::Tilt(int intakePivotMotorPort, int trayLimitPort) {
  trayTiltMotor = new pros::Motor(intakePivotMotorPort, false);

  trayLimitSwitch = new pros::ADIDigitalIn(trayLimitPort);
}

void Tilt::PivotUp() {
  double error = (constants::TRAY_SCORING_POS - GetTrayPosition()) / constants::TRAY_SCORING_POS;
  double power;

  if (error >= 0.2) {
    power = 200;
  } else if (error > 0.1 && error < 0.2) {
    power = 100;
  } else {
    power = 30;
  }

  trayTiltMotor->move_absolute(constants::TRAY_SCORING_POS, power);
}

void Tilt::PivotDown() {
  if (trayLimitSwitch->get_value() != 1) {
    trayTiltMotor->move(-127);
  } else {
    trayTiltMotor->move(-2);
  }
}

void Tilt::ManualControl(pros::Controller controller) {
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) == 1 ||
      controller.get_digital(pros::E_CONTROLLER_DIGITAL_B) == 1) {
    currentTrayState = trayState::Manual;
  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y) == 1) {
    currentTrayState = trayState::Up;
  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A) == 1) {
    currentTrayState = trayState::Down;
  }

  if (currentTrayState == trayState::Manual) {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) == 1) {
      Pivot(127);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B) == 1) {
      Pivot(-127);
    } else {
      Pivot(0);
    }
  }
}

int Tilt::IsTrayDown() {
  return trayLimitSwitch->get_value();
}

double Tilt::GetTrayPosition() {
  return trayTiltMotor->get_position();
}

void Tilt::Periodic() {
  // Reset sensors
  if (trayLimitSwitch->get_value() == 1) {
    trayTiltMotor->tare_position();
  }

  switch(currentTrayState) {
    case trayState::Manual:
    // Do nothing
    break;
    case trayState::Up:
      PivotUp();
    break;
    case trayState::Down:
      PivotDown();
    break;
  }
}

Tilt::~Tilt() {

}

void Tilt::Pivot(int speed) {
  trayTiltMotor->move(speed);
}
