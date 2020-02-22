#pragma once

#include "api.h"
#include "Auton.h"
#include "Robot.h"

class IntakeAction : public AutonAction {
private:
    Robot *m_robot;
public:
    enum class intakeStates {
      INTAKE,
      OUTTAKE_FULL,
      OUTTAKE_SLOW,
      OFF
    };

    intakeStates m_intakeState;

    IntakeAction(Robot *robot, intakeStates intakeState);

    void ActionInit();
    actionStatus Action();
    void ActionEnd();
};
