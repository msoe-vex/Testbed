#pragma once

#include "../Auton.h"
#include "../AutonActions/FollowPathAction.h"


class TestAuton : Autonomous {
public:
    TestAuton(Chassis chassis);
    void AddNodes();

private:
    Chassis * m_chassis;

    Node * m_driveToCubes = nullptr;
    Node * m_intakeCubes = nullptr;
    Node * m_spinInCircles = nullptr;
};


