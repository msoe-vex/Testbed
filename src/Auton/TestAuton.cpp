#include "TestAuton.h"
#include "../../include/PathManager.h"


TestAuton::TestAuton(Chassis chassis) : Autonomous("Test Auton") {
    m_chassis = chassis;
}

void TestAuton::AddNodes() {
    m_driveToCubes = new Node(1, new FollowPathAction(m_chassis, PathManager::GetInstance()->GetPath("TestPath"), 10, 3.25), new PrintAction("Starting Auto!"));
    m_intakeCubes = new Node(3, new PrintAction("This should intake the cubes"));
    m_spinInCircles = new Node(5, new PrintAction("Spinning!!!!!"));

    AddFirstNode(m_driveToCubes);

    m_driveToCubes->AddNext(m_intakeCubes);
    m_intakeCubes->AddNext(m_spinInCircles);
}
