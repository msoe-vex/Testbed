#include "FollowPathAction.h"


FollowPathAction::FollowPathAction(chassis * chassis, Path path, double maxAccel, double wheelDiameter, bool reversed,
                             double fixedLookahead, double pathCompletionTolerance, bool gradualStop) :
            m_controller(fixedLookahead, maxAccel, 0.01, path, reversed, pathCompletionTolerance, gradualStop,
                    wheelDiameter) {
    m_chassis = chassis;
}

void FollowPathAction::ActionInit() {

}

AutonAction::actionStatus FollowPathAction::Action() {
    auto command = m_controller.Update(TankOdometry::GetInstance()->GetPose(), pros::millis() / 1000.0);

    m_chassis->setSpeed(command.left, command.right);

    if(m_controller.isDone()) {
        return END;
    } else {
        return CONTINUE;
    }
}

void FollowPathAction::ActionEnd() {
    m_chassis->setSpeed(0, 0);
}
