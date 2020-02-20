#include "FollowPath.h"


FollowPath::FollowPath(Chassis chassis, Path path, double maxAccel, double wheelDiameter, bool reversed,
        double fixedLookahead, double pathCompletionTolerance, bool gradualStop) :
            m_controller(fixedLookahead, maxAccel, 0.01, path, reversed, pathCompletionTolerance, gradualStop,
                    wheelDiameter) {
    m_chassis = chassis;
}

void FollowPath::actionInit() {

}

AutonAction::actionStatus FollowPath::action() {
    auto command = m_controller.Update(TankOdometry::GetInstance()->GetPose(), pros::millis() / 1000.0);

    m_chassis.setVelocity(command.left, command.right);

    if(m_controller.isDone()) {
        return END;
    } else {
        return CONTINUE;
    }
}

void FollowPath::actionEnd() {
    m_chassis.setVelocity(0, 0);
}
