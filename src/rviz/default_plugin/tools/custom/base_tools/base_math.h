#ifndef TRAJ_TOOLS_BASE_MATH_H_
#define TRAJ_TOOLS_BASE_MATH_H_

#include <math.h>
#include "base_tools/base_type.h"

namespace traj_tools {
namespace common {

extern const Float kPi;                        ///PI
extern const Float kHalfPi;                    ///PI/2
extern const Float kQuarterPi;                 ///PI/4
extern const Float kThreeSecondsPi;            ///3*pi/2
extern const Float kTwoPi;                     ///2 * pi
extern const Float kSqrt2;                     ///sqrt(2.0)
extern const Float kOneThird;                  ///1/3

inline Float Asin(Float sin_theta) {
    if(sin_theta < -1.0) {
        return -kHalfPi;
    }
    else if(sin_theta < 1.0) {
        return asin(sin_theta);
    }
    else {
        return kHalfPi;
    }
}

inline Float Acos(Float cos_theta) {
    if(cos_theta < -1.0) {
        return kPi;
    }
    else if(cos_theta < 1.0) {
        return acos(cos_theta);
    }
    else {
        return 0.0;
    }
}

inline Float cubrt(Float data) {
    if(data > 0.0) {
        return pow(data, kOneThird);
    }
    else {
        return -(pow(-data, kOneThird));
    }
}

inline bool is_zero(Float value, Float eps = 0.0) {
    if(value <= eps && value >= -eps) {
        return true;
    }
    return false;
}
}       //namespace common
}       //namespace traj_tools

#endif // TRAJ_TOOLS_BASE_MATH_H_
