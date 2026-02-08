/**
 * @file discretization.hpp
 * @brief 连续系统离散化方法
 * 
 * 支持:
 * - ZOH (零阶保持)
 * - Tustin (双线性变换)
 * - Forward/Backward Euler
 */
#pragma once

#include "state_space.hpp"
#include <Eigen/Dense>
#include <cmath>

namespace gnc::libraries::control {

/**
 * @brief 离散化方法
 */
enum class DiscretizationMethod {
    ZOH,           ///< 零阶保持 (最精确)
    Tustin,        ///< 双线性变换 (Bilinear/Trapezoidal)
    ForwardEuler,  ///< 前向欧拉
    BackwardEuler  ///< 后向欧拉
};

/**
 * @brief 矩阵指数 (Padé 近似)
 * @param A 方阵
 * @param order Padé 阶数 (默认 6)
 * @return exp(A)
 */
inline MatrixX matrix_exp(const MatrixX& A, int order = 6) {
    int n = A.rows();
    MatrixX result = MatrixX::Identity(n, n);
    MatrixX A_power = MatrixX::Identity(n, n);
    double factorial = 1.0;
    
    for (int k = 1; k <= order; ++k) {
        A_power = A_power * A;
        factorial *= k;
        result += A_power / factorial;
    }
    
    return result;
}

/**
 * @brief ZOH 离散化
 * 
 * 对于连续系统 x' = Ac*x + Bc*u:
 * Ad = exp(Ac*T)
 * Bd = Ac^{-1} * (Ad - I) * Bc
 * 
 * @param sys 连续系统
 * @param dt 采样周期
 * @return 离散系统
 */
inline StateSpaceModel c2d_zoh(const StateSpaceModel& sys, double dt) {
    if (sys.isDiscrete()) {
        return sys;  // 已经是离散的
    }
    
    int n = sys.order();
    int m = sys.numInputs();
    
    // 使用矩阵指数计算 Ad
    MatrixX Ad = matrix_exp(sys.A() * dt, 13);
    
    // 计算 Bd
    // 使用增广矩阵方法避免求逆
    // [Ad Bd]     [Ac Bc]
    // [0  I ] = exp([ 0  0 ]*T)
    MatrixX Aaug = MatrixX::Zero(n + m, n + m);
    Aaug.block(0, 0, n, n) = sys.A() * dt;
    Aaug.block(0, n, n, m) = sys.B() * dt;
    
    MatrixX Maug = matrix_exp(Aaug, 13);
    MatrixX Bd = Maug.block(0, n, n, m);
    
    return StateSpaceModel(Ad, Bd, sys.C(), sys.D(), SystemType::Discrete);
}

/**
 * @brief Tustin 离散化 (双线性变换)
 * 
 * s = (2/T) * (z - 1) / (z + 1)
 * 
 * Ad = (I - A*T/2)^{-1} * (I + A*T/2)
 * Bd = (I - A*T/2)^{-1} * B * T
 * Cd = C
 * Dd = D + C * (I - A*T/2)^{-1} * B * T/2
 */
inline StateSpaceModel c2d_tustin(const StateSpaceModel& sys, double dt) {
    if (sys.isDiscrete()) {
        return sys;
    }
    
    int n = sys.order();
    MatrixX I = MatrixX::Identity(n, n);
    MatrixX Ahalf = sys.A() * (dt / 2.0);
    
    MatrixX IminusA = I - Ahalf;
    MatrixX IplusA = I + Ahalf;
    
    // 求逆
    MatrixX IminusA_inv = IminusA.inverse();
    
    MatrixX Ad = IminusA_inv * IplusA;
    MatrixX Bd = IminusA_inv * sys.B() * dt;
    MatrixX Dd = sys.D() + sys.C() * IminusA_inv * sys.B() * (dt / 2.0);
    
    return StateSpaceModel(Ad, Bd, sys.C(), Dd, SystemType::Discrete);
}

/**
 * @brief 前向欧拉离散化
 * 
 * Ad = I + A*T
 * Bd = B*T
 */
inline StateSpaceModel c2d_euler(const StateSpaceModel& sys, double dt) {
    if (sys.isDiscrete()) {
        return sys;
    }
    
    int n = sys.order();
    MatrixX I = MatrixX::Identity(n, n);
    
    MatrixX Ad = I + sys.A() * dt;
    MatrixX Bd = sys.B() * dt;
    
    return StateSpaceModel(Ad, Bd, sys.C(), sys.D(), SystemType::Discrete);
}

/**
 * @brief 后向欧拉离散化
 * 
 * Ad = (I - A*T)^{-1}
 * Bd = (I - A*T)^{-1} * B * T
 */
inline StateSpaceModel c2d_backward_euler(const StateSpaceModel& sys, double dt) {
    if (sys.isDiscrete()) {
        return sys;
    }
    
    int n = sys.order();
    MatrixX I = MatrixX::Identity(n, n);
    MatrixX IminusA = I - sys.A() * dt;
    MatrixX IminusA_inv = IminusA.inverse();
    
    MatrixX Ad = IminusA_inv;
    MatrixX Bd = IminusA_inv * sys.B() * dt;
    
    return StateSpaceModel(Ad, Bd, sys.C(), sys.D(), SystemType::Discrete);
}

/**
 * @brief 通用离散化函数
 */
inline StateSpaceModel c2d(const StateSpaceModel& sys, double dt, 
                           DiscretizationMethod method = DiscretizationMethod::ZOH) {
    switch (method) {
        case DiscretizationMethod::ZOH:
            return c2d_zoh(sys, dt);
        case DiscretizationMethod::Tustin:
            return c2d_tustin(sys, dt);
        case DiscretizationMethod::ForwardEuler:
            return c2d_euler(sys, dt);
        case DiscretizationMethod::BackwardEuler:
            return c2d_backward_euler(sys, dt);
        default:
            return c2d_zoh(sys, dt);
    }
}

/**
 * @brief 预卷曲 Tustin 离散化
 * 
 * 对于需要在特定频率精确匹配的系统
 * 
 * @param sys 连续系统
 * @param dt 采样周期
 * @param prewarp_freq 预卷曲频率 (rad/s)
 */
inline StateSpaceModel c2d_tustin_prewarp(const StateSpaceModel& sys, double dt, 
                                          double prewarp_freq) {
    // 预卷曲系数
    double alpha = prewarp_freq / std::tan(prewarp_freq * dt / 2.0);
    
    int n = sys.order();
    MatrixX I = MatrixX::Identity(n, n);
    MatrixX Ahalf = sys.A() / alpha;
    
    MatrixX IminusA = I - Ahalf;
    MatrixX IplusA = I + Ahalf;
    
    MatrixX IminusA_inv = IminusA.inverse();
    
    MatrixX Ad = IminusA_inv * IplusA;
    MatrixX Bd = IminusA_inv * sys.B() * (2.0 / alpha);
    MatrixX Dd = sys.D() + sys.C() * IminusA_inv * sys.B() / alpha;
    
    return StateSpaceModel(Ad, Bd, sys.C(), Dd, SystemType::Discrete);
}

} // namespace gnc::libraries::control
