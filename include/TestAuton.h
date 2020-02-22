#pragma once

#include "Robot.h"
#include "Auton.h"
#include "FollowPathAction.h"
#include "autonActions/DeployAction.h"
#include "autonActions/IntakeAction.h"
#include "PathManager.h"
#include "chassis.h"
#include "Math/Pose.h"
#include <Eigen/Dense>

using namespace Eigen;

class TestAuton : public Autonomous {
public:
    TestAuton(Robot *robot);
    void AddNodes();
    void AutonInit();

private:
    Robot *m_robot;

    Node *m_deploy = nullptr;
    Node *m_wallToSingleCube = nullptr;
    Node *m_waitIntakeSingleCube = nullptr;
    Node *m_singleCubeToFourStack = nullptr;
    Node *m_fourStackToThreeStack = nullptr;
    Node *m_threeStackToSingleCube = nullptr;
    Node *m_revSingleCubeToWallLineup = nullptr;
    Node *m_wallLineupToStack = nullptr;
};
