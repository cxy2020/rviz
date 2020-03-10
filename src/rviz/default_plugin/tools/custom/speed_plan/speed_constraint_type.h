#ifndef RVIZ_CUSTOM_SPEED_PLAN_SPEED_CONSTRAINT_TYPE_H_
#define RVIZ_CUSTOM_SPEED_PLAN_SPEED_CONSTRAINT_TYPE_H_

#include <algorithm>

#include "base_tools/static_capacity_vector.h"
#include "base_tools/base_type.h"

namespace traj_tools {

const Float kMaxLimit = 9999.9;
/**
 * @class SpeedConstraint
 *
 * @brief class containing speed constraint information
 */
class SpeedConstraint {
public:
    SpeedConstraint(Float max_linear_speed = kMaxLimit) :
        max_speed_(max_linear_speed) {}

    inline Float max_speed() const {
        return max_speed_;
    }

    inline void combine_speed(const SpeedConstraint& constraint) {
        if(max_speed_ > constraint.max_speed_) {
            max_speed_ = constraint.max_speed_;
        }
    }

private:
    Float max_speed_;
};
/**
 * @struct DistanceSpeedConstraint
 *
 * @brief Constraint the speed between the special distance on the path
 */
class DistanceSpeedConstraint : public SpeedConstraint {
public:
    DistanceSpeedConstraint() :
        SpeedConstraint(kMaxLimit),
        start_pos_(kMaxLimit),
        end_pos_(kMaxLimit) {}
    DistanceSpeedConstraint(Float max_linear_speed,
                            Float start_s,
                            Float end_s) :
        SpeedConstraint(max_linear_speed),
        start_pos_(start_s),
        end_pos_(end_s) {}

    DistanceSpeedConstraint(const SpeedConstraint& constraint,
                            Float start_s,
                            Float end_s) :
        SpeedConstraint(constraint),
        start_pos_(start_s),
        end_pos_(end_s) {}

    DistanceSpeedConstraint(const DistanceSpeedConstraint& constraint, Float start_s, Float end_s) :
        SpeedConstraint(constraint),
        start_pos_(start_s),
        end_pos_(end_s) {}

    inline void combine(const DistanceSpeedConstraint& constraint,
                        common::StaticCapacityVector<DistanceSpeedConstraint>& output) const {
        if(start_pos_ >= constraint.end_pos_) {
            output.push_back(constraint);
            output.push_back(*this);
            return;
        }
        else if(end_pos_ <= constraint.start_pos_) {
            output.push_back(*this);
            output.push_back(constraint);
            return;
        }

        Float left_s = 0.0;
        if(start_pos_ < constraint.start_pos_) {
            output.push_back(DistanceSpeedConstraint(*this, start_pos_, constraint.start_pos_));
            left_s = constraint.start_pos_;
        }
        else {
            output.push_back(DistanceSpeedConstraint(constraint, constraint.start_pos_, start_pos_));
            left_s = start_pos_;
        }
        if(end_pos_ < constraint.end_pos_) {
            output.push_back(DistanceSpeedConstraint(std::min(max_speed(), constraint.max_speed()),
                                                     left_s,
                                                     end_pos_));
            output.push_back(DistanceSpeedConstraint(constraint, end_pos_, constraint.end_pos_));
        }
        else {
            output.push_back(DistanceSpeedConstraint(std::min(max_speed(), constraint.max_speed()),
                                                     left_s,
                                                     constraint.end_pos_));
            output.push_back(DistanceSpeedConstraint(*this, constraint.end_pos_, end_pos_));
        }
    }

    inline bool is_before(const DistanceSpeedConstraint& constraint) const {
        if(start_pos_ > constraint.end_pos_) {
            return true;
        }
        return false;
    }

    inline bool is_behind(const DistanceSpeedConstraint& constraint) const {
        if(end_pos_ < constraint.start_pos_) {
            return true;
        }
        return false;
    }

    inline Float start_pos() const {
        return start_pos_;
    }

    inline Float end_pos() const {
        return end_pos_;
    }

private:
    friend class SpeedConstraintManager;
    Float start_pos_;
    Float end_pos_;
};

typedef common::StaticCapacityVector<SpeedConstraint> SpeedConstraintVec;
typedef common::StaticCapacityVector<SpeedConstraint>::Iterator SpeedConstraintIter;
typedef common::StaticCapacityVector<DistanceSpeedConstraint> DistanceSpeedConstraintVec;
typedef common::StaticCapacityVector<DistanceSpeedConstraint>::Iterator DistanceSpeedConstraintIter;

}       //namespace traj_tools

#endif // RVIZ_CUSTOM_SPEED_PLAN_SPEED_CONSTRAINT_TYPE_H_
