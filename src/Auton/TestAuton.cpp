#include "TestAuton.h"

TestAuton::TestAuton(Robot *robot) : Autonomous("Test Auton") {
    m_robot = robot;
}

void TestAuton::AutonInit() {
    Autonomous::AutonInit();
    auto position = PathManager::GetInstance()->GetPath("WallToSingleCube").getFirstWaypoint().position;
    auto angle = PathManager::GetInstance()->GetPath("WallToSingleCube").getFirstWaypoint().rotation;
    TankOdometry::GetInstance()->SetCurrentPose(Pose(Vector2d(position.getX(), position.getY()), Rotation2Dd(angle.getRadians())));
}

void TestAuton::AddNodes() {
    m_deploy = new Node(2, new DeployAction(m_robot));
    m_wallToSingleCube = new Node(5, new FollowPathAction(m_robot, PathManager::GetInstance()->GetPath("WallToSingleCube"), 50, 4),
                                     new IntakeAction(m_robot, IntakeAction::intakeStates::INTAKE),
                                     new PrintAction("Running WallToSingleCube"));
    m_waitIntakeSingleCube = new Node(5, new  WaitAction(3));

    /*
    m_singleCubeToFourStack;
    m_fourStackToThreeStack;
    m_threeStackToSingleCube;
    m_revSingleCubeToWallLineup;
    m_wallLineupToStack;
    */

/*

Node *m_singleCubeToFourStack = nullptr;
Node *m_fourStackToThreeStack = nullptr;
Node *m_threeStackToSingleCube = nullptr;
Node *m_revSingleCubeToWallLineup = nullptr;
Node *m_wallLineupToStack = nullptr;
    AddFirstNode(m_driveToCubes);

    m_driveToCubes->AddNext(m_intakeCubes);
    m_intakeCubes->AddNext(m_spinInCircles);
    */
}
