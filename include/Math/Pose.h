#pragma once

#include <Eigen/Dense>

using namespace Eigen;

struct Pose {
    Vector2d position;
    Rotation2Dd angle;

    Pose(Vector2d positionIn, Rotation2Dd angleIn) {
        position = positionIn;
        angle = angleIn;
    }

    Pose() {
        position = Vector2d();
        angle = Rotation2Dd();
    }
};
