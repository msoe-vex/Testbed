#include "TestAuton.h"


TestAuton::TestAuton(Chassis chassis) : Autonomous("Test Auton") {
    m_chassis = chassis;
}

void TestAuton::AddNodes() {
    m_driveToCubes = new Node(1, new FollowPathAction(m_chassis, PathsManager::))
}
