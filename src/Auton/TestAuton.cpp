#include "TestAuton.h"

TestAuton::TestAuton(chassis * chassis) : Autonomous("Test Auton") {
    m_chassis = chassis;
}

void TestAuton::AutonInit() {
    Autonomous::AutonInit();
    auto position = PathManager::GetInstance()->GetPath("WallToCubeStack3").getFirstWaypoint().position;
    auto angle = PathManager::GetInstance()->GetPath("WallToCubeStack3").getFirstWaypoint().rotation;
    TankOdometry::GetInstance()->SetCurrentPose(Pose(Vector2d(position.getX(), position.getY()), Rotation2Dd(angle.getRadians())));
}

void TestAuton::AddNodes() {
    m_driveToCubes = new Node(5, new FollowPathAction(m_chassis, PathManager::GetInstance()->GetPath("WallToCubeStack3"), 50, 4), new PrintAction("Starting Auto!"));
    m_intakeCubes = new Node(3, new PrintAction("This should intake the cubes"));
    m_spinInCircles = new Node(5, new PrintAction("Spinning!!!!!"));

    AddFirstNode(m_driveToCubes);

    m_driveToCubes->AddNext(m_intakeCubes);
    m_intakeCubes->AddNext(m_spinInCircles);
}
