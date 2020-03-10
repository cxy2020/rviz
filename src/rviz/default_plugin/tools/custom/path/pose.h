#ifndef TRAJ_TOOLS_POSE_H_
#define TRAJ_TOOLS_POSE_H_

#include "base_tools/base_type.h"

namespace traj_tools {

struct Position {
    Float x;
    Float y;
    Float z;
};

struct Orientation {
    Float x;
    Float y;
    Float z;
    Float w;
};

struct Pose {
    Position position;
    Orientation orientation;
};

}       //namespace traj_tools

#endif // TRAJ_TOOLS_POSE_H_
