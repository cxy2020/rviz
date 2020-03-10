#ifndef TRAJ_TOOLS_PATH_PATH_LINE_H_
#define TRAJ_TOOLS_PATH_PATH_LINE_H_

#include "path/path_2d.h"

namespace traj_tools {

class Line : public Path2d
{
public:
    Line() {}

    Line(const Vec2& start, const Vec2& end);

    void Reset(const Vec2& pos0, Float theta0, Float length);

    virtual Float GetPos(Float s,
                         Float& x,
                         Float& y,
                         Float& k,
                         Float& theta) const;
};

}       //namespace traj_tools

#endif // TRAJ_TOOLS_PATH_PATH_LINE_H_
