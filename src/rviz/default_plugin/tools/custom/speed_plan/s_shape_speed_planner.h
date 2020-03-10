#ifndef TRAJ_TOOLS_SPEED_PLAN_S_SHAPE_SPEED_PANNER_H_
#define TRAJ_TOOLS_SPEED_PLAN_S_SHAPE_SPEED_PANNER_H_

#include <assert.h>
#include <limits>
#include <math.h>

#include "speed_plan/speed_constraint_type.h"
#include "base_tools/base_type.h"

namespace traj_tools {

class SShapeSpeedPlanner {
public:
    enum StopResult {
        STOP_SUCCESS = 0,
        STOP_SUCESS_WITH_ADJUST,
        STOP_EXACTLY,
        STOP_FAIL
    };

    struct PlanParam {
        inline PlanParam(Float a0, Float v0, Float s0);

        inline void copy(const PlanParam& copy);

        inline bool has_only_last_dec_phase() const;

        inline Float start_s() const {
            return s[0];
        }

        inline Float end_s() const {
            return s[3];
        }

        inline Float total_time() const {
            return t[3];
        }

        inline void set_start(Float a0, Float v0, Float s0) {
            a[0] = a0;
            v[0] = v0;
            s[0] = s0;
            t[0] = 0.0;
        }

        inline bool is_exceed(const DistanceSpeedConstraint& speed_constraint, Float max_jerk) const;

        inline int find_speed(Float max_v, Float max_jerk, int index, bool* is_rising, Float* pos_nodes) const;

        inline bool find_stop_constraint(const DistanceSpeedConstraintVec& speed_constraints,
                                         DistanceSpeedConstraintIter& exceed_iter) const;


        Float t[4];
        Float a[4];
        Float v[5];
        Float s[4];
    };

    SShapeSpeedPlanner();

    SShapeSpeedPlanner(Float max_acc, Float max_dec, Float max_jerk);

    inline void SetKinematicParam(Float max_acc, Float max_dec, Float max_jerk);

    void acc_to_expect(Float v_expect, Float* t, Float* a, Float* v) const;

    Float get_try_stop_distance(Float a0, Float v0, Float s0);

    void Plan(Float s0,
              Float v0,
              Float a0,
              Float target_s,
              const DistanceSpeedConstraintVec& speed_constraints,
              Float max_emergency_dec,
              Float time_step,
              Float& s_end,
              Float& v_end,
              Float& a_end,
              Float min_v);

    StopResult PlanStop(Float s0,
                        Float v0,
                        Float a0,
                        Float time_length,
                        Float min_v,
                        Float& s_end,
                        Float& v_end,
                        Float& a_end,
                        Float& stop_distance);
    inline Float max_dec() const {
        return m_max_dec;
    }
private:

    void acc_to_expect(Float v_expect,
                       PlanParam& plan_param) const;

    void PlanStopAt(Float s0,
                    Float v0,
                    Float a0,
                    Float target_s,
                    Float max_emergency_dec,
                    Float time_step,
                    Float& s_end,
                    Float& v_end,
                    Float& a_end,
                    Float min_v = 0.0);

    void AccToStopAt(Float s0,
                     Float v0,
                     Float a0,
                     Float target_s,
                     Float time_step,
                     Float& s_end,
                     Float& v_end,
                     Float& a_end,
                     Float min_v = 0.0);

    inline static void GetMotionStatus(const PlanParam& plan_param,
                                       Float max_jerk,
                                       Float t,
                                       Float& a,
                                       Float& v);


    inline static void GetMotionStatus(const PlanParam& plan_param,
                                       Float max_jerk,
                                       Float t,
                                       Float& a,
                                       Float& v,
                                       Float& s);

    static inline bool is_need_uniform_acc(Float delta_v, Float expect_delta_v);

    static inline bool is_need_uniform_dec(Float delta_v, Float expect_delta_v);

    static inline Float recalc_peak_acc(Float a0, Float expect_delta_v, Float max_jerk) {
        Float acc_square = a0 * a0 * 0.5 + max_jerk * expect_delta_v;
        if(acc_square > 0.0) {
            return sqrt(acc_square);
        }
        else {
            return 0.0;
        }
    }

    static inline Float recalc_peak_dec(Float a0, Float expect_delta_v, Float max_jerk) {
        Float acc_square = a0 * a0 * 0.5 - max_jerk * expect_delta_v;
        if(acc_square > 0.0) {
            return -sqrt(acc_square);
        }
        else {
            return 0.0;
        }
    }

    inline void set_kinematic_param(Float max_acc, Float max_dec, Float max_jerk);

    inline void stop_with_new_param(Float a0,
                                    Float v0,
                                    Float s0,
                                    Float new_max_dec,
                                    Float new_max_jerk,
                                    Float min_v,
                                    Float time_step,
                                    Float& a_end,
                                    Float& v_end,
                                    Float& s_end);

    inline void reset_max_dec(Float max_dec);

    inline void plan_3_phases_dec_with_jerk(Float max_jerk,
                                            Float s0,
                                            Float v0,
                                            Float a0,
                                            Float target_s,
                                            Float min_dec,
                                            Float max_dec,
                                            Float time_step,
                                            Float min_v,
                                            Float& s_end,
                                            Float& v_end,
                                            Float& a_end);

    Float m_max_acc;
    Float m_max_dec;
    Float m_max_jerk;
    Float m_half_acc;                      ///temporary variable，acc/2
    Float m_half_dec;                      ///temporary variable，dec/2
    Float m_half_jerk;                     ///temporary variable，jerk/2
    Float m_sixth_jerk;                    ///temporary variable，jerk/6
    Float m_t_max_acc_to_zero;             ///temporary variable，time comsumed for declining acc to zero
    Float m_d_speed_max_acc_to_zero;       ///temporary variable，velocity increment for declining acc to zero
    Float m_dd_distance_max_acc_to_zero;   ///temporary variable，distance increment for declining acc to zero: max_acc * t^2 / 2 + jerk * t^3 / 6
    Float m_t_max_dec_to_zero;             ///temporary variable，time comsumed for declining dec to zero
    Float m_d_speed_max_dec_to_zero;       ///temporary variable，velocity increment for declining dec to zero
    Float m_dd_distance_max_dec_to_zero;   ///temporary variable，distance increment for declining acc to zero: max_dec * t^2 / 2 + jerk * t^3 / 6

    Float m_init_max_acc;                  ///user configured max acc
    Float m_init_max_dec;                  ///user configured max dec
    Float m_init_max_jerk;                 ///user configured max jerk
};
//设定S型速度规划的运动参数
void SShapeSpeedPlanner::SetKinematicParam(Float max_acc, Float max_dec, Float max_jerk) {
    m_init_max_acc = max_acc;
    m_init_max_dec = max_dec;
    m_init_max_jerk = max_jerk;
    set_kinematic_param(max_acc, max_dec, max_jerk);
}
//设定S型速度规划的运动参数，不对初始值进行设置
void SShapeSpeedPlanner::set_kinematic_param(Float max_acc, Float max_dec, Float max_jerk) {
    m_max_acc = Abs(max_acc);
    m_max_dec = Abs(max_dec);
    m_max_jerk = Abs(max_jerk);
    assert(m_max_jerk != 0.0);
    m_half_acc = 0.5 * m_max_acc;
    m_half_dec = 0.5 * m_max_dec;
    m_half_jerk = 0.5 * max_jerk;
    m_sixth_jerk = max_jerk / 6.0;

    if(m_max_jerk >= std::numeric_limits<Float>::max()) {
        m_t_max_acc_to_zero = 0.0;
        m_d_speed_max_acc_to_zero = 0.0;
        m_dd_distance_max_acc_to_zero = 0.0;
        m_t_max_dec_to_zero = 0.0;
        m_d_speed_max_dec_to_zero = 0.0;
        m_dd_distance_max_dec_to_zero = 0.0;
        return;
    }
    //计算从最大加速度减速变化到最大加速度为零，对应的参数
    m_t_max_acc_to_zero = m_max_acc / m_max_jerk;
    Float t_square = m_t_max_acc_to_zero * m_t_max_acc_to_zero;
    m_d_speed_max_acc_to_zero = m_max_acc * m_t_max_acc_to_zero * 0.5;
    m_dd_distance_max_acc_to_zero = m_half_acc * t_square - m_sixth_jerk * m_t_max_acc_to_zero * t_square;
    //计算从最大减速度减速变化到最大减速度为零，对应的参数
    m_t_max_dec_to_zero = m_max_dec / m_max_jerk;
    t_square = m_t_max_dec_to_zero * m_t_max_dec_to_zero;
    m_d_speed_max_dec_to_zero = -m_max_dec * m_t_max_dec_to_zero * 0.5;
    m_dd_distance_max_dec_to_zero = -m_half_dec * t_square + m_sixth_jerk * m_t_max_dec_to_zero * t_square;
}
//重新设置最大减速度的大小
void SShapeSpeedPlanner::reset_max_dec(Float max_dec) {
    m_max_dec = Abs(max_dec);
    m_half_dec = 0.5 * m_max_dec;

    if(m_max_jerk >= std::numeric_limits<Float>::max()) {
        m_t_max_dec_to_zero = 0.0;
        m_d_speed_max_dec_to_zero = 0.0;
        m_dd_distance_max_dec_to_zero = 0.0;
        return;
    }
    //计算从最大减速度减速变化到最大减速度为零，对应的参数
    m_t_max_dec_to_zero = m_max_dec / m_max_jerk;
    Float t_square = m_t_max_dec_to_zero * m_t_max_dec_to_zero;
    m_d_speed_max_dec_to_zero = -m_max_dec * m_t_max_dec_to_zero * 0.5;
    m_dd_distance_max_dec_to_zero = -m_half_dec * t_square + m_sixth_jerk * m_t_max_dec_to_zero * t_square;
}

}       //namespace traj_tools
#endif // TRAJ_TOOLS_SPEED_PLAN_S_SHAPE_SPEED_PANNER_H_
