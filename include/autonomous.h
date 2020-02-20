#pragma once

#include "API.h"

class autonomous {
public:
  enum class autonMode {
    Left_Red_Autonomous,
    Right_Red_Autonomous,
    Left_Blue_Autonomous,
    Right_Blue_Autonomous,
    Programming_Skills
  };

  autonomous(autonMode auton);

  ~autonomous();
};
