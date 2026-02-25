/**
 * @file optimization.hpp
 * @brief 优化算法 - 1D/多维无约束优化和 QP 接口
 * 
 * 包含:
 * - 黄金分割搜索 (1D)
 * - Nelder-Mead 单纯形法 (无导数)
 * - 梯度下降
 * - QP 求解器接口 (供第三方库实现)
 * 
 * @note 复杂优化建议使用成熟开源库: OSQP, NLopt, Ceres
 */
#pragma once

#include "eigen_types.hpp"
#include "calculus.hpp"
#include <functional>
#include <algorithm>
#include <limits>

namespace gnc::math {

// ==================== 优化结果 ====================

/**
 * @brief 优化结果 (标量)
 */
struct OptResult1D {
    double x_opt = 0.0;      ///< 最优点
    double f_opt = 0.0;      ///< 最优值
    int iterations = 0;      ///< 迭代次数
    bool converged = false;  ///< 是否收敛
};

/**
 * @brief 优化结果 (向量)
 */
struct OptResult {
    VectorX x_opt;           ///< 最优点
    double f_opt = 0.0;      ///< 最优值
    int iterations = 0;      ///< 迭代次数
    int f_evals = 0;         ///< 函数评估次数
    bool converged = false;  ///< 是否收敛
};

/**
 * @brief QP 求解结果
 */
struct QpResult {
    VectorX x;               ///< 最优解
    double obj = 0.0;        ///< 目标函数值
    bool success = false;    ///< 是否成功
    int iterations = 0;      ///< 迭代次数
    std::string message;     ///< 状态信息
};

// ==================== 黄金分割搜索 ====================

/**
 * @brief 黄金分割搜索 (1D 优化)
 * 
 * 在区间 [a, b] 内寻找一元函数的极小值点
 * 
 * @param f 目标函数
 * @param a 左端点
 * @param b 右端点
 * @param tol 容差
 * @param max_iter 最大迭代次数
 */
inline OptResult1D golden_section_search(const ScalarFunc& f,
                                          double a, double b,
                                          double tol = 1e-8,
                                          int max_iter = 100) {
    constexpr double phi = 1.618033988749895;  // 黄金比例
    constexpr double resphi = 2.0 - phi;       // 1/phi
    
    OptResult1D result;
    
    double c = b - resphi * (b - a);
    double d = a + resphi * (b - a);
    double fc = f(c);
    double fd = f(d);
    
    for (int i = 0; i < max_iter; ++i) {
        result.iterations = i + 1;
        
        if (std::abs(b - a) < tol) {
            result.converged = true;
            break;
        }
        
        if (fc < fd) {
            b = d;
            d = c;
            fd = fc;
            c = b - resphi * (b - a);
            fc = f(c);
        } else {
            a = c;
            c = d;
            fc = fd;
            d = a + resphi * (b - a);
            fd = f(d);
        }
    }
    
    result.x_opt = (a + b) / 2.0;
    result.f_opt = f(result.x_opt);
    return result;
}

/**
 * @brief Brent 1D 优化 (黄金分割 + 抛物线插值)
 */
inline OptResult1D brent_minimize(const ScalarFunc& f,
                                   double a, double b,
                                   double tol = 1e-8,
                                   int max_iter = 100) {
    constexpr double CGOLD = 0.3819660;
    constexpr double ZEPS = 1e-10;
    
    OptResult1D result;
    
    double x = (a + b) / 2.0;
    double w = x, v = x;
    double fx = f(x), fw = fx, fv = fx;
    double d = 0.0, e = 0.0;
    
    for (int i = 0; i < max_iter; ++i) {
        result.iterations = i + 1;
        
        double xm = (a + b) / 2.0;
        double tol1 = tol * std::abs(x) + ZEPS;
        double tol2 = 2.0 * tol1;
        
        if (std::abs(x - xm) <= tol2 - 0.5 * (b - a)) {
            result.converged = true;
            break;
        }
        
        double u;
        if (std::abs(e) > tol1) {
            // 抛物线插值
            double r = (x - w) * (fx - fv);
            double q = (x - v) * (fx - fw);
            double p = (x - v) * q - (x - w) * r;
            q = 2.0 * (q - r);
            if (q > 0.0) p = -p;
            q = std::abs(q);
            double etemp = e;
            e = d;
            
            if (std::abs(p) >= std::abs(0.5 * q * etemp) ||
                p <= q * (a - x) || p >= q * (b - x)) {
                e = (x >= xm) ? a - x : b - x;
                d = CGOLD * e;
            } else {
                d = p / q;
                u = x + d;
                if (u - a < tol2 || b - u < tol2) {
                    d = (xm >= x) ? tol1 : -tol1;
                }
            }
        } else {
            e = (x >= xm) ? a - x : b - x;
            d = CGOLD * e;
        }
        
        u = (std::abs(d) >= tol1) ? x + d : x + ((d >= 0) ? tol1 : -tol1);
        double fu = f(u);
        
        if (fu <= fx) {
            if (u >= x) a = x; else b = x;
            v = w; w = x; x = u;
            fv = fw; fw = fx; fx = fu;
        } else {
            if (u < x) a = u; else b = u;
            if (fu <= fw || w == x) {
                v = w; w = u;
                fv = fw; fw = fu;
            } else if (fu <= fv || v == x || v == w) {
                v = u; fv = fu;
            }
        }
    }
    
    result.x_opt = x;
    result.f_opt = fx;
    return result;
}

// ==================== Nelder-Mead 单纯形法 ====================

/**
 * @brief Nelder-Mead 单纯形优化
 * 
 * 无导数多维优化方法，适用于光滑或非光滑问题
 * 
 * @param f 目标函数
 * @param x0 初始猜测
 * @param tol 容差
 * @param max_iter 最大迭代次数
 * @param initial_step 初始单纯形大小
 */
inline OptResult nelder_mead(const VectorToScalarFunc& f,
                              const VectorX& x0,
                              double tol = 1e-8,
                              int max_iter = 1000,
                              double initial_step = 1.0) {
    OptResult result;
    int n = x0.size();
    
    // Nelder-Mead 参数
    constexpr double alpha = 1.0;   // 反射
    constexpr double gamma = 2.0;   // 扩张
    constexpr double rho = 0.5;     // 收缩
    constexpr double sigma = 0.5;   // 缩减
    
    // 初始化单纯形 (n+1 个顶点)
    std::vector<VectorX> simplex(n + 1);
    std::vector<double> fvals(n + 1);
    
    simplex[0] = x0;
    fvals[0] = f(x0);
    result.f_evals = 1;
    
    for (int i = 0; i < n; ++i) {
        simplex[i + 1] = x0;
        simplex[i + 1](i) += initial_step;
        fvals[i + 1] = f(simplex[i + 1]);
        result.f_evals++;
    }
    
    // 主循环
    for (int iter = 0; iter < max_iter; ++iter) {
        result.iterations = iter + 1;
        
        // 排序顶点
        std::vector<int> order(n + 1);
        for (int i = 0; i <= n; ++i) order[i] = i;
        std::sort(order.begin(), order.end(), [&](int a, int b) {
            return fvals[a] < fvals[b];
        });
        
        // 重排
        std::vector<VectorX> new_simplex(n + 1);
        std::vector<double> new_fvals(n + 1);
        for (int i = 0; i <= n; ++i) {
            new_simplex[i] = simplex[order[i]];
            new_fvals[i] = fvals[order[i]];
        }
        simplex = new_simplex;
        fvals = new_fvals;
        
        // 收敛检查
        double range = fvals[n] - fvals[0];
        if (range < tol) {
            result.converged = true;
            break;
        }
        
        // 计算重心 (除最差点外)
        VectorX centroid = VectorX::Zero(n);
        for (int i = 0; i < n; ++i) {
            centroid += simplex[i];
        }
        centroid /= n;
        
        // 反射
        VectorX xr = centroid + alpha * (centroid - simplex[n]);
        double fr = f(xr);
        result.f_evals++;
        
        if (fr >= fvals[0] && fr < fvals[n - 1]) {
            simplex[n] = xr;
            fvals[n] = fr;
        } else if (fr < fvals[0]) {
            // 扩张
            VectorX xe = centroid + gamma * (xr - centroid);
            double fe = f(xe);
            result.f_evals++;
            
            if (fe < fr) {
                simplex[n] = xe;
                fvals[n] = fe;
            } else {
                simplex[n] = xr;
                fvals[n] = fr;
            }
        } else {
            // 收缩
            VectorX xc = centroid + rho * (simplex[n] - centroid);
            double fc = f(xc);
            result.f_evals++;
            
            if (fc < fvals[n]) {
                simplex[n] = xc;
                fvals[n] = fc;
            } else {
                // 缩减
                for (int i = 1; i <= n; ++i) {
                    simplex[i] = simplex[0] + sigma * (simplex[i] - simplex[0]);
                    fvals[i] = f(simplex[i]);
                    result.f_evals++;
                }
            }
        }
    }
    
    // 找最优
    int best = 0;
    for (int i = 1; i <= n; ++i) {
        if (fvals[i] < fvals[best]) best = i;
    }
    
    result.x_opt = simplex[best];
    result.f_opt = fvals[best];
    return result;
}

// ==================== 梯度下降 ====================

/**
 * @brief 梯度下降优化
 * 
 * @param f 目标函数
 * @param x0 初始猜测
 * @param lr 学习率
 * @param tol 容差
 * @param max_iter 最大迭代
 */
inline OptResult gradient_descent(const VectorToScalarFunc& f,
                                   const VectorX& x0,
                                   double lr = 0.01,
                                   double tol = 1e-8,
                                   int max_iter = 1000) {
    OptResult result;
    VectorX x = x0;
    
    for (int i = 0; i < max_iter; ++i) {
        result.iterations = i + 1;
        
        VectorX grad = numerical_gradient(f, x);
        result.f_evals += 2 * x.size();  // 中心差分
        
        if (grad.norm() < tol) {
            result.converged = true;
            break;
        }
        
        x -= lr * grad;
    }
    
    result.x_opt = x;
    result.f_opt = f(x);
    result.f_evals++;
    return result;
}

/**
 * @brief 带回溯线搜索的梯度下降
 */
inline OptResult gradient_descent_armijo(const VectorToScalarFunc& f,
                                          const VectorX& x0,
                                          double tol = 1e-8,
                                          int max_iter = 1000) {
    OptResult result;
    VectorX x = x0;
    
    constexpr double c = 1e-4;      // Armijo 参数
    constexpr double rho = 0.5;     // 回溯因子
    
    for (int i = 0; i < max_iter; ++i) {
        result.iterations = i + 1;
        
        double fx = f(x);
        VectorX grad = numerical_gradient(f, x);
        result.f_evals += 1 + 2 * x.size();
        
        if (grad.norm() < tol) {
            result.converged = true;
            result.x_opt = x;
            result.f_opt = fx;
            return result;
        }
        
        // 线搜索
        double alpha = 1.0;
        VectorX d = -grad;
        double slope = grad.dot(d);
        
        while (f(x + alpha * d) > fx + c * alpha * slope) {
            alpha *= rho;
            result.f_evals++;
            if (alpha < 1e-16) break;
        }
        
        x += alpha * d;
    }
    
    result.x_opt = x;
    result.f_opt = f(x);
    result.f_evals++;
    return result;
}

// ==================== QP 求解器接口 ====================

/**
 * @brief QP 求解器接口 (抽象类)
 * 
 * 求解二次规划问题:
 * min  0.5 * x^T * H * x + f^T * x
 * s.t. A_eq * x = b_eq
 *      lb <= x <= ub
 * 
 * @note 这是一个接口类，由用户提供具体实现或绑定第三方库 (如 OSQP)
 */
class IQpSolver {
public:
    virtual ~IQpSolver() = default;
    
    /**
     * @brief 求解 QP 问题
     * @param H 二次项系数 (n x n, 对称正半定)
     * @param f 线性项系数 (n)
     * @param A_eq 等式约束矩阵 (m x n)
     * @param b_eq 等式约束右端 (m)
     * @param lb 下界 (n)
     * @param ub 上界 (n)
     */
    virtual QpResult solve(const MatrixX& H, const VectorX& f,
                           const MatrixX& A_eq, const VectorX& b_eq,
                           const VectorX& lb, const VectorX& ub) = 0;
    
    /**
     * @brief 简化求解 (无约束)
     */
    virtual QpResult solve_unconstrained(const MatrixX& H, const VectorX& f) {
        int n = f.size();
        return solve(H, f, 
                     MatrixX::Zero(0, n), VectorX::Zero(0),
                     VectorX::Constant(n, -std::numeric_limits<double>::infinity()),
                     VectorX::Constant(n, std::numeric_limits<double>::infinity()));
    }
};

/**
 * @brief 简单 QP 求解 (无约束)
 * 
 * 仅适用于 H 正定的情况: x* = -H^{-1} * f
 */
inline QpResult solve_qp_unconstrained(const MatrixX& H, const VectorX& f) {
    QpResult result;
    
    Eigen::LLT<MatrixX> llt(H);
    if (llt.info() != Eigen::Success) {
        result.success = false;
        result.message = "H is not positive definite";
        return result;
    }
    
    result.x = llt.solve(-f);
    // 目标函数值: 0.5 * x^T * H * x + f^T * x
    double xHx = result.x.dot(H * result.x);
    double fx = f.dot(result.x);
    result.obj = 0.5 * xHx + fx;
    result.success = true;
    result.iterations = 1;
    return result;
}

} // namespace gnc::math
