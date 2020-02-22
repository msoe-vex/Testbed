#pragma once

#include "Auton.h"
#include "Robot.h"
#include "Lift.h"
#include "Intake.h"
#include "AdaptivePursuit.h"

class DeployAction : public AutonAction {
private:
  Robot *m_robot;
  Timer timer;

public:
  DeployAction(Robot *robot);

  void ActionInit();

  actionStatus Action();

  void ActionEnd();
};
