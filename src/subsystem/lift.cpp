#include "Lift.h"

lift::lift(int leftLiftMotorPort, int rightLiftMotorPort) {
  leftLiftMotor = new pros::Motor(leftLiftMotorPort, false);
  rightLiftMotor = new pros::Motor(rightLiftMotorPort, true);
}

double lift::getPosition() {
  return (leftLiftMotor->get_position() + rightLiftMotor->get_position()) / 2;
}

double lift::getVelocity() {
  return leftLiftMotor->get_actual_velocity();
}

void lift::setLiftState(pros::Controller controller) {
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) == 1 ||
      controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) == 1) {
    setLiftState(liftState::Manual, 0);
  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) == 1) {
    setLiftState(liftState::LowGoal, constants::LOW_GOAL_POS);
  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) == 1) {
    setLiftState(liftState::FourStack, constants::FOUR_STACK_POS);
  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) == 1) {
    setLiftState(liftState::MediumGoal, constants::MID_GOAL_POS);
  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) == 1) {
    setLiftState(liftState::HighGoal, constants::HIGH_GOAL_POS);
  }

  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y) == 1 &&
      !isStateScoring) {
    isStateScoring = true;
  } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y) == 1 &&
      isStateScoring) {
    isStateScoring = false;
  }
}

void lift::setLiftState(liftState liftState) {
  currentLiftState = liftState;
}

void lift::setLiftState(liftState liftState, double setpoint) {
  currentLiftState = liftState;
  currentSetpoint = setpoint;
}

void lift::periodic(pros::Controller controller) {
  switch(currentLiftState) {
    case liftState::Manual:
      manualControl(controller);
    break;
    case liftState::LowGoal:
    case liftState::FourStack:
    case liftState::MediumGoal:
    case liftState::HighGoal:
    case liftState::VariableSetpointPID:
      setPIDPosition(currentSetpoint);
    break;
  }
}

lift::~lift() {
  free(leftLiftMotor);
  free(rightLiftMotor);

  leftLiftMotor = 0;
  rightLiftMotor = 0;
}

// Private methods

void lift::manualControl(pros::Controller controller) {
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) == 1) {
    setPower(127);
  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) == 1) {
    setPower(-127);
  } else {
    setPower(4);
  }
}

void lift::setPower(int liftPower) {
  leftLiftMotor->move(liftPower);
  rightLiftMotor->move(liftPower);
}

void lift::setPIDPosition(double position) {
  leftLiftMotor->move_absolute(position, constants::LIFT_MAX_VEL);
  rightLiftMotor->move_absolute(position, constants::LIFT_MAX_VEL);
}

void lift::setPIDVelocity(double liftVelocity) {
  leftLiftMotor->move_velocity(liftVelocity);
  rightLiftMotor->move_velocity(liftVelocity);
}
