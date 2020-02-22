#include "autonActions/IntakeAction.h"


IntakeAction::IntakeAction(Robot *robot, intakeStates intakeState) {
  m_robot = robot;
  m_intakeState = intakeState;
}

void IntakeAction::ActionInit() {
  switch(m_intakeState) {
    case intakeStates::INTAKE:
      m_robot->intake.SetSpeed(127, 127);
    break;
    case intakeStates::OUTTAKE_FULL:
      m_robot->intake.SetSpeed(-127, -127);
    break;
    case intakeStates::OUTTAKE_SLOW:
      m_robot->intake.SetSpeed(-40, -40);
    break;
    case intakeStates::OFF:
      m_robot->intake.SetSpeed(0, 0);
    break;
  }
}

AutonAction::actionStatus IntakeAction::Action() {
  return END;
}

void IntakeAction::ActionEnd() {
}
