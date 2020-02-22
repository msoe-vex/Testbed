#include "autonActions/DeployAction.h"


DeployAction::DeployAction(Robot *robot) {
  m_robot = robot;
}

void DeployAction::ActionInit() {
  timer.Start();
}

AutonAction::actionStatus DeployAction::Action() {
  // Code to deploy robot here

  if(timer.Get() > 1) { // Terminate at 3 seconds
      return END;
  } else {
      return CONTINUE;
  }
}

void DeployAction::ActionEnd() {

}
