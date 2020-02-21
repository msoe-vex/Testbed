#include "FollowPathAction.h"


FollowPathAction::FollowPathAction(Chassis *chassis, Path path, double maxAccel, double wheelDiameter, bool reversed,
                             double fixedLookahead, double pathCompletionTolerance, bool gradualStop) :
            m_controller(fixedLookahead, maxAccel, 0.01, path, reversed, pathCompletionTolerance, gradualStop,
                    wheelDiameter) {
    m_chassis = chassis;
}

void FollowPathAction::ActionInit() {

}

AutonAction::actionStatus FollowPathAction::Action() {
    auto command = m_controller.Update(TankOdometry::GetInstance()->GetPose(), pros::millis() / 1000.0);

    m_chassis->SetSpeed(command.left * (127.0/100.0), command.right * (127.0/100.0));

    if(m_controller.isDone()) {
        return END;
    } else {
        return CONTINUE;
    }
}

void FollowPathAction::ActionEnd() {
    m_chassis->SetSpeed(0, 0);
}
