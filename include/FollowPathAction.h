#pragma once

#include "Auton.h"
#include "Robot.h"
#include "AdaptivePursuit.h"

class FollowPathAction : public AutonAction {
private:
    Robot *m_robot;
    AdaptivePursuit m_controller;
public:
    FollowPathAction(Robot *robot, Path path, double maxAccel, double wheelDiameter, bool reversed = false,
                     double fixedLookahead = 10, double pathCompletionTolerance = 0.1, bool gradualStop = true);

    void ActionInit();
    actionStatus Action();
    void ActionEnd();
};
