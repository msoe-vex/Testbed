#include "Lift.h"

Lift::Lift(int liftMotorPort, int liftLimitPort) {
  liftMotor = new pros::Motor(liftMotorPort, false);

  liftLimitSwitch = new pros::ADIDigitalIn(liftLimitPort);
}

void Lift::ManualControl(pros::Controller controller) {
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) == 1 ||
      controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) == 1) {
    SetLiftState(liftState::Manual, 0);
  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) == 1) {
    SetLiftState(liftState::LowGoal, constants::LOW_GOAL_POS);
  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) == 1) {
    SetLiftState(liftState::MediumGoal, constants::MID_GOAL_POS);
  }

  if (currentLiftState == liftState::Manual) {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) == 1) {
      SetPower(127);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) == 1) {
      SetPower(-127);
    } else {
      SetPower(5);
    }
  }
}

double Lift::GetPosition() {
  return liftMotor->get_position();
}

double Lift::GetVelocity() {
  return liftMotor->get_actual_velocity();
}

void Lift::SetLiftState(liftState liftState) {
  currentLiftState = liftState;
}

void Lift::SetLiftState(liftState liftState, double setpoint) {
  currentLiftState = liftState;
  currentSetpoint = setpoint;
}

void Lift::Periodic() {
  // Reset sensors
  if (liftLimitSwitch->get_value() == 1) {
    liftMotor->tare_position();
  }

  switch(currentLiftState) {
    case liftState::Manual:
    // Do nothing
    break;
    case liftState::LowGoal:
    case liftState::MediumGoal:
    case liftState::VariableSetpointPID:
      SetPIDPosition(currentSetpoint);
    break;
  }
}

Lift::~Lift() {
  free(liftMotor);

  liftMotor = 0;
}

// Private methods

void Lift::SetPower(int liftPower) {
  liftMotor->move(liftPower);
}

void Lift::SetPIDPosition(double position) {
  liftMotor->move_absolute(position, constants::LIFT_MAX_VEL);
}

void Lift::SetPIDVelocity(double liftVelocity) {
  liftMotor->move_velocity(liftVelocity);
}
