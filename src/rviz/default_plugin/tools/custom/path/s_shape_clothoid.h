#ifndef TRAJ_TOOLS_PATH_S_SHAPE_CLOTHOID_H_
#define TRAJ_TOOLS_PATH_S_SHAPE_CLOTHOID_H_

#include "path/clothoid.h"

namespace traj_tools {

class SShapeClothoid : public Path2d {
public:
    enum SplitResult {
        NO_SPLIT,               ///No split
        SPLIT_AT_BEGIN,         ///Splited at the start
        SPLIT_AT_MIDDLE,        ///Splited at the middle
        NOT_ENOUGH_SLOPE,       ///With correct direction but not enough slop
        SPLIT_AT_END,           ///Splited at the end
    };

    SShapeClothoid();

    SShapeClothoid(const Vec2& pos0,
                   Float theta0,
                   Float k0,
                   Float theta_t,
                   Float max_k,
                   Float c);

    SShapeClothoid(const Vec2& pos0,
                   Float theta0,
                   Float s[4],
                   Float k[4],
                   Float c);

    void Reset(const Vec2& pos0,
               Float theta0,
               Float k0,
               Float theta_t,
               Float max_k,
               Float c);

    void Reset(const Vec2& pos0,
               Float theta0,
               Float end_s,
               Float s[4],
               Float k[4],
               Float c);

    void Reset(const Vec2& pos0,
               Float theta0,
               Float s[4],
               Float k[4],
               Float c);

    virtual Float GetPos(Float s,
                         Float& x,
                         Float& y,
                         Float& k,
                         Float& theta) const;

    void Translate(const Vec2& delta_pos);
private:
    SShapeClothoid* CutAtDistance(Float s);

    SShapeClothoid* CopyToDistance(const SShapeClothoid& source, Float end_s);

    inline Float first_two_clothoids_length() {
        return m_clothoids[0].length() + m_clothoids[1].length();
    }

    inline void update_parent_members() {
        length_ = m_clothoids[0].length_ + m_clothoids[1].length_ + m_clothoids[2].length_;
        start_pos_ = m_clothoids[0].start_pos_;
        end_pos_ = m_clothoids[2].end_pos_;
        start_dir_ = m_clothoids[0].start_dir_;
        end_dir_ = m_clothoids[2].end_dir_;
        is_dir_computed_ = (m_clothoids[0].is_dir_computed_ && m_clothoids[2].is_dir_computed_);
        start_k_ = m_clothoids[0].start_k_;
        end_k_ = m_clothoids[2].end_k_;
        start_theta_ = m_clothoids[0].start_theta_;
        end_theta_ = m_clothoids[2].end_theta_;
        is_theta_computed_ = (m_clothoids[0].is_theta_computed_ && m_clothoids[2].is_theta_computed_);
    }
    Clothoid m_clothoids[3];

    friend class CompositeClothoid;
};

}       //namespace traj_tool
#endif // TRAJ_TOOLS_PATH_S_SHAPE_CLOTHOID_H_
