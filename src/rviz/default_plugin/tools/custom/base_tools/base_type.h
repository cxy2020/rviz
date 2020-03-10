#ifndef TRAJ_TOOLS_COMMON_BASE_TYPE_H_
#define TRAJ_TOOLS_COMMON_BASE_TYPE_H_

#define Abs abs

#ifdef USE_FLOAT
#define Float float
#define Vec2 Eigen::Vector2f
#else
#define Float double
#define Vec2 Eigen::Vector2d
#endif

#endif // TRAJ_TOOLS_COMMON_BASE_TYPE_H_
