/**
 * @file calculus.hpp
 * @brief 数值积分与微分 - ODE 积分器和梯度计算
 * 
 * 包含:
 * - RK4 固定步长积分
 * - RK45 自适应步长积分 (Dormand-Prince)
 * - 数值梯度 (中心差分)
 * - 数值 Jacobian
 * - 数值 Hessian
 * 
 * @note 所有函数均为无状态纯函数
 */
#pragma once

#include "eigen_types.hpp"
#include <functional>
#include <cmath>

namespace gnc::math {

// ==================== 类型定义 ====================

/// 标量 ODE 右端函数: dy/dt = f(t, y)
using OdeScalarFunc = std::function<double(double t, double y)>;

/// 向量 ODE 右端函数: dy/dt = f(t, y)
using OdeVectorFunc = std::function<VectorX(double t, const VectorX& y)>;

/// 标量函数: y = f(x)
using ScalarFunc = std::function<double(double x)>;

/// 向量到标量函数: y = f(x)
using VectorToScalarFunc = std::function<double(const VectorX& x)>;

/// 向量到向量函数: y = f(x)
using VectorToVectorFunc = std::function<VectorX(const VectorX& x)>;

// ==================== RK4 积分 ====================

/**
 * @brief RK4 单步积分 (标量)
 * @param f ODE 右端函数
 * @param t 当前时间
 * @param y 当前状态
 * @param h 时间步长
 * @return 下一时刻状态
 */
inline double rk4_step(const OdeScalarFunc& f, double t, double y, double h) {
    double k1 = f(t, y);
    double k2 = f(t + h/2, y + h*k1/2);
    double k3 = f(t + h/2, y + h*k2/2);
    double k4 = f(t + h, y + h*k3);
    return y + h * (k1 + 2*k2 + 2*k3 + k4) / 6.0;
}

/**
 * @brief RK4 单步积分 (向量)
 * @param f ODE 右端函数
 * @param t 当前时间
 * @param y 当前状态向量
 * @param h 时间步长
 * @return 下一时刻状态向量
 */
inline VectorX rk4_step(const OdeVectorFunc& f, double t, const VectorX& y, double h) {
    VectorX k1 = f(t, y);
    VectorX k2 = f(t + h/2, y + h*k1/2);
    VectorX k3 = f(t + h/2, y + h*k2/2);
    VectorX k4 = f(t + h, y + h*k3);
    return y + h * (k1 + 2*k2 + 2*k3 + k4) / 6.0;
}

// ==================== RK45 自适应积分 ====================

/**
 * @brief RK45 积分结果
 */
struct Rk45Result {
    VectorX y_new;       ///< 5 阶解
    VectorX y_err;       ///< 误差估计 (5阶 - 4阶)
    double error_norm;   ///< 误差范数
};

/**
 * @brief RK45 (Dormand-Prince) 单步积分
 * 
 * Butcher 表:
 * 0    |
 * 1/5  | 1/5
 * 3/10 | 3/40    9/40
 * 4/5  | 44/45   -56/15   32/9
 * 8/9  | 19372/6561  -25360/2187  64448/6561  -212/729
 * 1    | 9017/3168   -355/33      46732/5247   49/176   -5103/18656
 * 1    | 35/384      0            500/1113     125/192  -2187/6784   11/84
 * 
 * @return 包含 5 阶解和误差估计的结果
 */
inline Rk45Result rk45_step(const OdeVectorFunc& f, double t, const VectorX& y, double h) {
    // Dormand-Prince 系数
    constexpr double a21 = 1.0/5.0;
    constexpr double a31 = 3.0/40.0, a32 = 9.0/40.0;
    constexpr double a41 = 44.0/45.0, a42 = -56.0/15.0, a43 = 32.0/9.0;
    constexpr double a51 = 19372.0/6561.0, a52 = -25360.0/2187.0, a53 = 64448.0/6561.0, a54 = -212.0/729.0;
    constexpr double a61 = 9017.0/3168.0, a62 = -355.0/33.0, a63 = 46732.0/5247.0, a64 = 49.0/176.0, a65 = -5103.0/18656.0;
    constexpr double a71 = 35.0/384.0, a73 = 500.0/1113.0, a74 = 125.0/192.0, a75 = -2187.0/6784.0, a76 = 11.0/84.0;
    
    // 5阶权重
    constexpr double b1 = 35.0/384.0, b3 = 500.0/1113.0, b4 = 125.0/192.0, b5 = -2187.0/6784.0, b6 = 11.0/84.0;
    // 4阶权重 (用于误差估计)
    constexpr double e1 = 71.0/57600.0, e3 = -71.0/16695.0, e4 = 71.0/1920.0;
    constexpr double e5 = -17253.0/339200.0, e6 = 22.0/525.0, e7 = -1.0/40.0;
    
    VectorX k1 = f(t, y);
    VectorX k2 = f(t + h/5, y + h*a21*k1);
    VectorX k3 = f(t + 3*h/10, y + h*(a31*k1 + a32*k2));
    VectorX k4 = f(t + 4*h/5, y + h*(a41*k1 + a42*k2 + a43*k3));
    VectorX k5 = f(t + 8*h/9, y + h*(a51*k1 + a52*k2 + a53*k3 + a54*k4));
    VectorX k6 = f(t + h, y + h*(a61*k1 + a62*k2 + a63*k3 + a64*k4 + a65*k5));
    VectorX k7 = f(t + h, y + h*(a71*k1 + a73*k3 + a74*k4 + a75*k5 + a76*k6));
    
    Rk45Result result;
    result.y_new = y + h * (b1*k1 + b3*k3 + b4*k4 + b5*k5 + b6*k6);
    result.y_err = h * (e1*k1 + e3*k3 + e4*k4 + e5*k5 + e6*k6 + e7*k7);
    result.error_norm = result.y_err.norm();
    
    return result;
}

/**
 * @brief ODE 积分选项
 */
struct OdeOptions {
    double rtol = 1e-6;        ///< 相对容差
    double atol = 1e-9;        ///< 绝对容差
    double h_min = 1e-12;      ///< 最小步长
    double h_max = 0.1;        ///< 最大步长
    int max_steps = 100000;    ///< 最大步数
    double safety = 0.9;       ///< 安全因子
};

/**
 * @brief ODE 积分结果
 */
struct OdeResult {
    std::vector<double> t;     ///< 时间点
    std::vector<VectorX> y;    ///< 状态向量序列
    bool success = true;       ///< 是否成功
    int n_steps = 0;           ///< 实际步数
    int n_reject = 0;          ///< 拒绝步数
};

/**
 * @brief 自适应步长 ODE 积分
 * @param f ODE 右端函数
 * @param t_span 时间区间 [t0, tf]
 * @param y0 初始状态
 * @param opts 积分选项
 * @return 积分结果
 */
inline OdeResult integrate_ode(const OdeVectorFunc& f, 
                               std::pair<double, double> t_span,
                               const VectorX& y0,
                               const OdeOptions& opts = OdeOptions()) {
    OdeResult result;
    result.t.push_back(t_span.first);
    result.y.push_back(y0);
    
    double t = t_span.first;
    double tf = t_span.second;
    VectorX y = y0;
    
    // 初始步长估计
    double h = std::min(opts.h_max, (tf - t) / 10.0);
    
    while (t < tf && result.n_steps < opts.max_steps) {
        if (t + h > tf) h = tf - t;
        
        auto step = rk45_step(f, t, y, h);
        
        // 计算误差尺度
        double scale = opts.atol + opts.rtol * std::max(y.norm(), step.y_new.norm());
        double err_ratio = step.error_norm / scale;
        
        if (err_ratio <= 1.0) {
            // 接受步
            t += h;
            y = step.y_new;
            result.t.push_back(t);
            result.y.push_back(y);
            result.n_steps++;
            
            // 增加步长
            if (err_ratio > 0) {
                h = std::min(opts.h_max, opts.safety * h * std::pow(err_ratio, -0.2));
            }
        } else {
            // 拒绝步，减小步长
            h = std::max(opts.h_min, opts.safety * h * std::pow(err_ratio, -0.25));
            result.n_reject++;
        }
        
        if (h < opts.h_min) {
            result.success = false;
            break;
        }
    }
    
    return result;
}

/**
 * @brief 固定步长 ODE 积分 (RK4)
 */
inline OdeResult integrate_ode_fixed(const OdeVectorFunc& f,
                                     double t0, double tf, double dt,
                                     const VectorX& y0) {
    OdeResult result;
    result.t.push_back(t0);
    result.y.push_back(y0);
    
    double t = t0;
    VectorX y = y0;
    
    while (t < tf - dt/2) {
        y = rk4_step(f, t, y, dt);
        t += dt;
        result.t.push_back(t);
        result.y.push_back(y);
        result.n_steps++;
    }
    
    return result;
}

// ==================== 数值微分 ====================

/**
 * @brief 数值梯度 (中心差分)
 * @param f 标量函数
 * @param x 求值点
 * @param h 差分步长 (默认 sqrt(eps) * |x|)
 * @return 梯度向量
 */
inline VectorX numerical_gradient(const VectorToScalarFunc& f, 
                                   const VectorX& x, 
                                   double h = -1.0) {
    int n = x.size();
    VectorX grad(n);
    
    if (h < 0) {
        h = std::sqrt(std::numeric_limits<double>::epsilon()) * std::max(1.0, x.norm());
    }
    
    VectorX x_plus = x, x_minus = x;
    for (int i = 0; i < n; ++i) {
        x_plus(i) = x(i) + h;
        x_minus(i) = x(i) - h;
        grad(i) = (f(x_plus) - f(x_minus)) / (2.0 * h);
        x_plus(i) = x(i);
        x_minus(i) = x(i);
    }
    
    return grad;
}

/**
 * @brief 数值 Jacobian (中心差分)
 * @param f 向量到向量函数
 * @param x 求值点
 * @param h 差分步长
 * @return Jacobian 矩阵 J[i][j] = ∂f_i/∂x_j
 */
inline MatrixX numerical_jacobian(const VectorToVectorFunc& f,
                                   const VectorX& x,
                                   double h = -1.0) {
    if (h < 0) {
        h = std::sqrt(std::numeric_limits<double>::epsilon()) * std::max(1.0, x.norm());
    }
    
    VectorX f0 = f(x);
    int m = f0.size();
    int n = x.size();
    MatrixX J(m, n);
    
    VectorX x_plus = x, x_minus = x;
    for (int j = 0; j < n; ++j) {
        x_plus(j) = x(j) + h;
        x_minus(j) = x(j) - h;
        VectorX f_plus = f(x_plus);
        VectorX f_minus = f(x_minus);
        J.col(j) = (f_plus - f_minus) / (2.0 * h);
        x_plus(j) = x(j);
        x_minus(j) = x(j);
    }
    
    return J;
}

/**
 * @brief 数值 Hessian (中心差分)
 * @param f 标量函数
 * @param x 求值点
 * @param h 差分步长
 * @return Hessian 矩阵 H[i][j] = ∂²f/∂x_i∂x_j
 */
inline MatrixX numerical_hessian(const VectorToScalarFunc& f,
                                  const VectorX& x,
                                  double h = -1.0) {
    if (h < 0) {
        h = std::pow(std::numeric_limits<double>::epsilon(), 1.0/3.0) * std::max(1.0, x.norm());
    }
    
    int n = x.size();
    MatrixX H(n, n);
    double f0 = f(x);
    
    VectorX x_i = x, x_j = x, x_ij = x;
    
    for (int i = 0; i < n; ++i) {
        x_i(i) = x(i) + h;
        double f_i_plus = f(x_i);
        x_i(i) = x(i) - h;
        double f_i_minus = f(x_i);
        x_i(i) = x(i);
        
        // 对角项
        H(i, i) = (f_i_plus - 2*f0 + f_i_minus) / (h * h);
        
        // 非对角项
        for (int j = i + 1; j < n; ++j) {
            x_ij = x;
            x_ij(i) += h;
            x_ij(j) += h;
            double f_pp = f(x_ij);
            
            x_ij(i) -= 2*h;
            double f_mp = f(x_ij);
            
            x_ij(j) -= 2*h;
            double f_mm = f(x_ij);
            
            x_ij(i) += 2*h;
            double f_pm = f(x_ij);
            
            H(i, j) = (f_pp - f_mp - f_pm + f_mm) / (4.0 * h * h);
            H(j, i) = H(i, j);  // 对称
        }
    }
    
    return H;
}

/**
 * @brief 数值导数 (标量函数)
 */
inline double numerical_derivative(const ScalarFunc& f, double x, double h = -1.0) {
    if (h < 0) {
        h = std::sqrt(std::numeric_limits<double>::epsilon()) * std::max(1.0, std::abs(x));
    }
    return (f(x + h) - f(x - h)) / (2.0 * h);
}

/**
 * @brief 数值二阶导数 (标量函数)
 */
inline double numerical_second_derivative(const ScalarFunc& f, double x, double h = -1.0) {
    if (h < 0) {
        h = std::pow(std::numeric_limits<double>::epsilon(), 1.0/3.0) * std::max(1.0, std::abs(x));
    }
    return (f(x + h) - 2*f(x) + f(x - h)) / (h * h);
}

} // namespace gnc::math
