#ifndef TRAJ_TOOLS_PATH_COMPOSITE_CLOTHOID_H_
#define TRAJ_TOOLS_PATH_COMPOSITE_CLOTHOID_H_

#include "path/line.h"
#include "path/s_shape_clothoid.h"
#include "base_tools/static_capacity_vector.h"

namespace traj_tools {

class CompositeClothoid : public Path2d
{
public:
    CompositeClothoid();

    CompositeClothoid(const Vec2& pos0,
                      Float theta0,
                      Float k0,
                      Float x_t,
                      Float y_t,
                      Float theta_t,
                      Float dir_x,
                      Float dir_y,
                      Float max_k,
                      Float max_c,
                      Float &max_yaw);

    void Reset(const Vec2& pos0,
               Float theta0,
               Float k0,
               Float x_t,
               Float y_t,
               Float theta_t,
               Float dir_x,
               Float dir_y,
               Float max_k,
               Float max_c,
               Float &max_yaw);

    virtual Float GetPos(Float s,
                         Float& x,
                         Float& y,
                         Float& k,
                         Float& theta) const;
private:
    inline Float calc_project(Float x, Float y, Float dir_x, Float dir_y) {
        return x * dir_x + y * dir_y;
    }

    inline Float calc_vertical_project(Float x, Float y, Float dir_x, Float dir_y) {
        return x * dir_y - y * dir_x;
    }

    inline static bool is_positive(Float value) {
        return (value > 0.0);
    }

    inline static bool is_negative(Float value) {
        return (value < 0.0);
    }

    inline void use_s_clothoid(int index, Float x_t, Float y_t, Float dir_x, Float dir_y) {
        Float length = 0.0;
        for(int i = 0; i <= index; ++i) {
            m_path_list.push_back(&m_s_clothoids[i]);
            length += m_s_clothoids[i].length();
        }
        Float delta_x = x_t - m_s_clothoids[index].end_pos()[0];
        Float delta_y = y_t - m_s_clothoids[index].end_pos()[1];
        Float line_length = calc_project(delta_x, delta_y, dir_x, dir_y);
        if(line_length > 0.0) {
            m_line.Reset(m_s_clothoids[index].end_pos(),
                         m_s_clothoids[index].end_theta(),
                         line_length);
            m_path_list.push_back(&m_line);
        }
    }

    inline void update_parent_members();

    common::StaticCapacityVector<Path2d*> m_path_list;
    SShapeClothoid m_s_clothoids[2];
    Line m_line_between_s_clothoids;      /// line between the 2 s-clothoids when neccessary
    Line m_line;                          /// line after all the s-clothoids and line coincides with the tracked path
};

}       //namespace traj_tools

#endif // TRAJ_TOOLS_PATH_COMPOSITE_CLOTHOID_H_
