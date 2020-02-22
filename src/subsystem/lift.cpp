#include "Lift.h"

Lift::Lift(int leftLiftMotorPort, int rightLiftMotorPort, int liftLimitPort) {
  leftLiftMotor = new pros::Motor(leftLiftMotorPort, false);
  rightLiftMotor = new pros::Motor(rightLiftMotorPort, true);

  liftLimitSwitch = new pros::ADIDigitalIn(liftLimitPort);
}

void Lift::ManualControl(pros::Controller controller) {
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) == 1 ||
      controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) == 1) {
    SetLiftState(liftState::Manual, 0);
  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) == 1) {
    SetLiftState(liftState::LowGoal, constants::LOW_GOAL_POS);
  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) == 1) {
    SetLiftState(liftState::FourStack, constants::FOUR_STACK_POS);
  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) == 1) {
    SetLiftState(liftState::MediumGoal, constants::MID_GOAL_POS);
  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) == 1) {
    SetLiftState(liftState::HighGoal, constants::HIGH_GOAL_POS);
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
  return (leftLiftMotor->get_position() + rightLiftMotor->get_position()) / 2;
}

double Lift::GetVelocity() {
  return leftLiftMotor->get_actual_velocity();
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
  if (liftLimitSwitch->get_value() == 0) {
    leftLiftMotor->tare_position();
    rightLiftMotor->tare_position();
  }

  switch(currentLiftState) {
    case liftState::Manual:
    // Do nothing
    break;
    case liftState::LowGoal:
    case liftState::FourStack:
    case liftState::MediumGoal:
    case liftState::HighGoal:
    case liftState::VariableSetpointPID:
      SetPIDPosition(currentSetpoint);
    break;
  }
}

Lift::~Lift() {
  free(leftLiftMotor);
  free(rightLiftMotor);

  leftLiftMotor = 0;
  rightLiftMotor = 0;
}

// Private methods

void Lift::SetPower(int liftPower) {
  leftLiftMotor->move(liftPower);
  rightLiftMotor->move(liftPower);
}

void Lift::SetPIDPosition(double position) {
  leftLiftMotor->move_absolute(position, constants::LIFT_MAX_VEL);
  rightLiftMotor->move_absolute(position, constants::LIFT_MAX_VEL);
}

void Lift::SetPIDVelocity(double liftVelocity) {
  leftLiftMotor->move_velocity(liftVelocity);
  rightLiftMotor->move_velocity(liftVelocity);
}
