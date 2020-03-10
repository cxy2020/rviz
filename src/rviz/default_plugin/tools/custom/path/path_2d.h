#ifndef TRAJ_TOOLS_PATH_PATH_2D_H_
#define TRAJ_TOOLS_PATH_PATH_2D_H_

#include <Eigen/Dense>

#include "path/pose.h"
#include "base_tools/base_type.h"
#include "base_tools/base_math.h"

namespace traj_tools {

class Path2d
{
public:
    Path2d() : length_(0.0)
      , start_pos_(0.0, 0.0)
      , end_pos_(0.0, 0.0)
      , start_dir_(1.0, 0.0)
      , end_dir_(1.0, 0.0)
      , is_dir_computed_(false)
      , start_k_(0.0)
      , end_k_(0.0)
      , start_theta_(0.0)
      , end_theta_(0.0)
      , is_theta_computed_(true) {}

    virtual ~Path2d() {}

    inline Float length() const {
        return length_;
    }

    inline const Vec2& start_pos() const {
        return start_pos_;
    }

    inline const Vec2& end_pos() const {
        return end_pos_;
    }

    inline const Vec2& start_dir() {
        if(is_dir_computed_) {
            return start_dir_;
        }
        else {
            compute_dir();
            return start_dir_;
        }
    }

    inline const Vec2& end_dir() {
        if(is_dir_computed_) {
            return end_dir_;
        }
        else {
            compute_dir();
            return end_dir_;
        }
    }

    inline Float start_k() const {
        return start_k_;
    }

    inline Float end_k() const {
        return end_k_;
    }

    inline Float start_theta() {
        if(is_theta_computed_) {
            return start_theta_;
        }
        else {
            compute_theta();
            return start_theta_;
        }
    }

    inline Float end_theta() {
        if(is_theta_computed_) {
            return end_theta_;
        }
        else {
            compute_theta();
            return end_theta_;
        }
    }

    Pose GetPose(Float length) const;

    virtual Float GetPos(Float s,
                         Float& x,
                         Float& y,
                         Float& k,
                         Float& theta) const = 0;

protected:
    inline void copy(const Path2d& path) {
        length_ = path.length_;
        start_pos_ = path.start_pos_;
        end_pos_ = path.end_pos_;
        start_dir_ = path.start_dir_;
        end_dir_ = path.end_dir_;
        is_dir_computed_ = path.is_dir_computed_;
        start_k_ = path.start_k_;
        end_k_ = path.end_k_;
        start_theta_ = path.start_theta_;
        end_theta_ = path.end_theta_;
        is_theta_computed_ = path.is_theta_computed_;
    }

    inline void copy_start(const Path2d& path) {
        start_pos_ = path.start_pos_;
        start_dir_ = path.start_dir_;
        is_dir_computed_ &= path.is_dir_computed_;
        start_k_ = path.start_k_;
        start_theta_ = path.start_theta_;
        is_theta_computed_ &= path.is_theta_computed_;
    }

    inline void copy_end(const Path2d& path) {
        end_pos_ = path.end_pos_;
        end_dir_ = path.end_dir_;
        is_dir_computed_ &= path.is_dir_computed_;
        end_k_ = path.end_k_;
        end_theta_ = path.end_theta_;
        is_theta_computed_ &= path.is_theta_computed_;
    }

    Float length_;
    Vec2 start_pos_;
    Vec2 end_pos_;
    Vec2 start_dir_;
    Vec2 end_dir_;
    bool is_dir_computed_;
    Float start_k_;
    Float end_k_;
    Float start_theta_;
    Float end_theta_;
    bool is_theta_computed_;

private:
    inline void compute_theta() {
        Float theta = common::Acos(start_dir_[0]);
        if(start_dir_[1] >= 0.0) {
            start_theta_ = end_theta_ = theta;
        }
        else {
            start_theta_ = end_theta_ = -theta;
        }
        is_theta_computed_ = true;
    }
    inline void compute_dir() {
        start_dir_[0] = cos(start_theta_);
        start_dir_[1] = sin(start_theta_);
        end_dir_[0] = cos(end_theta_);
        end_dir_[1] = sin(end_theta_);
        is_dir_computed_ = true;
    }
};

}       //namespace traj_tools

#endif // TRAJ_TOOLS_PATH_PATH_2D_H_
