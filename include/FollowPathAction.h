#pragma once

#include "Auton.h"
#include "chassis.h"
#include "AdaptivePursuit.h"

class FollowPathAction : public AutonAction {
private:
    chassis * m_chassis;
    AdaptivePursuit m_controller;
public:
    FollowPathAction(chassis * chassis, Path path, double maxAccel, double wheelDiameter, bool reversed = false,
                     double fixedLookahead = 10, double pathCompletionTolerance = 0.1, bool gradualStop = true);

    void ActionInit();
    actionStatus Action();
    void ActionEnd();
};