#ifndef TRAJ_TOOLS_COMMON_EQUATION_SOLVER_H_
#define TRAJ_TOOLS_COMMON_EQUATION_SOLVER_H_

#include "base_tools/base_type.h"

namespace traj_tools {
namespace common {

/**
 * @brief 方程求解类，包含求解多项式的函数接口
 */
class EquationSolver {
public:
    enum SolverResult {
        NUMBEROUS_SOLUTION = -1,
        UNSOLVABLE = 0,
        ONE = 1,
        TWO = 2,
        THREE = 3,
        FOUR = 4
    };
    /**
     * @brief 解线性方程a * x + b = 0
     * @param a x的系数
     * @param b 常数项
     * @return 方程的解
     */
    static SolverResult SolveLinear(Float a, Float b, Float& root);
    /**
     * @brief 解一元二次方程，a * x^2 + b * x + c = 0
     * @param a：方程二次项参数
     * @param b：方程一次项参数
     * @param c：方程常数项参数
     * @param root[]：方程的解的指针
     * @return 方程的解的个数
     */
    static SolverResult SolveQuadratic(Float a,
                                       Float b,
                                       Float c,
                                       Float root[]);

    /**
     * @brief 解一元三次方程，a * x^3 + b * x^2 + c * x + d =0
     * @param a：方程三次次项参数
     * @param b：方程二次项参数
     * @param c：方程一次项参数
     * @param d：方程常数项参数
     * @param root[]：方程的解的指针
     * @return 方程的解的个数
     */
    static SolverResult SolveCubic(Float a,
                                   Float b,
                                   Float c,
                                   Float d,
                                   Float root[]);

    /**
     * @brief 解一元四次方程，a * x^4 + b * x^3 + c * x^2 + d * x + e = 0
     * @param a：一元一次方程四次项参数
     * @param b：一元一次方程三次项参数
     * @param c：一元一次方程二次项参数
     * @param d：一元一次方程一次项参数
     * @param e：一元一次方程常次项参数
     * @param root[]：方程的解的指针
     * @return 方程的解的个数
     */
    static SolverResult SolveQuartic(Float a,
                                     Float b,
                                     Float c,
                                     Float d,
                                     Float e,
                                     Float root[]);

private:
    static const Float kError;
    static const Float kSqrtThree;
};

}       //namespace common
}       //namespace traj_tools
#endif // TRAJ_TOOLS_COMMON_EQUATION_SOLVER_H_
