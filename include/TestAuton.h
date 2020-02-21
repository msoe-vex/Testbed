#pragma once

#include "Auton.h"
#include "FollowPathAction.h"
#include "PathManager.h"
#include "chassis.h"


class TestAuton : public Autonomous {
public:
    TestAuton(chassis * chassis);
    void AddNodes();

private:
    chassis * m_chassis;

    Node * m_driveToCubes = nullptr;
    Node * m_intakeCubes = nullptr;
    Node * m_spinInCircles = nullptr;
};
