#pragma once

#include "../Auton.h"

class FollowPathAction : AutonAction {
private:
    Chassis * chassis;
    AdaptivePursuit m_controller;
public:
    FollowPathAction(Chassis chassis, Path path, double maxAccel, double wheelDiameter, bool reversed = false,
                     double fixedLookahead = 10, double pathCompletionTolerance = 0.1, bool gradualStop = true);

    void actionInit();
    actionStatus action();
    void actionEnd();
};
