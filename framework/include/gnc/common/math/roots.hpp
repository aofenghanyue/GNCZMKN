/**
 * @file roots.hpp
 * @brief 方程求根 - 非线性方程和多项式求根
 * 
 * 包含:
 * - 牛顿-拉夫森法 (标量/向量)
 * - 二分法
 * - Brent 方法
 * - 多项式求根 (伴随矩阵法)
 * 
 * @note 所有函数均为无状态纯函数
 */
#pragma once

#include "eigen_types.hpp"
#include "calculus.hpp"
#include <functional>
#include <complex>
#include <cmath>

namespace gnc::math {

// ==================== 求根结果 ====================

/**
 * @brief 标量求根结果
 */
struct RootResult {
    double root = 0.0;       ///< 根
    double residual = 0.0;   ///< 残差 |f(root)|
    int iterations = 0;      ///< 迭代次数
    bool converged = false;  ///< 是否收敛
};

/**
 * @brief 向量求根结果
 */
struct RootResultND {
    VectorX root;            ///< 根
    double residual = 0.0;   ///< 残差 ||f(root)||
    int iterations = 0;      ///< 迭代次数
    bool converged = false;  ///< 是否收敛
};

// ==================== 牛顿-拉夫森法 ====================

/**
 * @brief 牛顿-拉夫森法求根 (标量，带导数)
 * @param f 函数
 * @param df 导数
 * @param x0 初始猜测
 * @param tol 容差
 * @param max_iter 最大迭代次数
 */
inline RootResult newton_raphson(const ScalarFunc& f, 
                                  const ScalarFunc& df,
                                  double x0,
                                  double tol = 1e-10,
                                  int max_iter = 100) {
    RootResult result;
    double x = x0;
    
    for (int i = 0; i < max_iter; ++i) {
        double fx = f(x);
        double dfx = df(x);
        
        if (std::abs(dfx) < 1e-30) {
            break;  // 导数太小，可能在极值点
        }
        
        double dx = fx / dfx;
        x -= dx;
        result.iterations = i + 1;
        
        if (std::abs(dx) < tol || std::abs(fx) < tol) {
            result.converged = true;
            break;
        }
    }
    
    result.root = x;
    result.residual = std::abs(f(x));
    return result;
}

/**
 * @brief 牛顿-拉夫森法求根 (标量，数值导数)
 */
inline RootResult newton_raphson(const ScalarFunc& f,
                                  double x0,
                                  double tol = 1e-10,
                                  int max_iter = 100) {
    auto df = [&f](double x) { return numerical_derivative(f, x); };
    return newton_raphson(f, df, x0, tol, max_iter);
}

/**
 * @brief 多维牛顿法求根
 * @param f 向量函数 (求 f(x) = 0)
 * @param x0 初始猜测
 * @param tol 容差
 * @param max_iter 最大迭代次数
 */
inline RootResultND newton_raphson_nd(const VectorToVectorFunc& f,
                                      const VectorX& x0,
                                      double tol = 1e-10,
                                      int max_iter = 100) {
    RootResultND result;
    VectorX x = x0;
    
    for (int i = 0; i < max_iter; ++i) {
        VectorX fx = f(x);
        MatrixX J = numerical_jacobian(f, x);
        
        // 求解 J * dx = -fx
        VectorX dx = J.colPivHouseholderQr().solve(-fx);
        x += dx;
        result.iterations = i + 1;
        
        if (dx.norm() < tol || fx.norm() < tol) {
            result.converged = true;
            break;
        }
    }
    
    result.root = x;
    result.residual = f(x).norm();
    return result;
}

// ==================== 二分法 ====================

/**
 * @brief 二分法求根
 * @param f 函数 (假设在 [a,b] 上异号)
 * @param a 左端点
 * @param b 右端点
 * @param tol 容差
 * @param max_iter 最大迭代次数
 */
inline RootResult bisection(const ScalarFunc& f,
                            double a, double b,
                            double tol = 1e-10,
                            int max_iter = 100) {
    RootResult result;
    
    double fa = f(a);
    double fb = f(b);
    
    if (fa * fb > 0) {
        // 同号，无法确定根
        result.converged = false;
        return result;
    }
    
    for (int i = 0; i < max_iter; ++i) {
        double c = (a + b) / 2.0;
        double fc = f(c);
        result.iterations = i + 1;
        
        if (std::abs(fc) < tol || (b - a) / 2.0 < tol) {
            result.root = c;
            result.residual = std::abs(fc);
            result.converged = true;
            return result;
        }
        
        if (fa * fc < 0) {
            b = c;
            fb = fc;
        } else {
            a = c;
            fa = fc;
        }
    }
    
    result.root = (a + b) / 2.0;
    result.residual = std::abs(f(result.root));
    return result;
}

// ==================== Brent 方法 ====================

/**
 * @brief Brent 方法求根
 * 
 * 结合二分法、割线法和逆二次插值的优点
 */
inline RootResult brent(const ScalarFunc& f,
                        double a, double b,
                        double tol = 1e-10,
                        int max_iter = 100) {
    RootResult result;
    
    double fa = f(a);
    double fb = f(b);
    
    if (fa * fb > 0) {
        result.converged = false;
        return result;
    }
    
    if (std::abs(fa) < std::abs(fb)) {
        std::swap(a, b);
        std::swap(fa, fb);
    }
    
    double c = a, fc = fa;
    bool mflag = true;
    double s = 0, d = 0;
    
    for (int i = 0; i < max_iter; ++i) {
        result.iterations = i + 1;
        
        if (std::abs(fb) < tol) {
            result.root = b;
            result.residual = std::abs(fb);
            result.converged = true;
            return result;
        }
        
        if (std::abs(b - a) < tol) {
            result.root = b;
            result.residual = std::abs(fb);
            result.converged = true;
            return result;
        }
        
        if (std::abs(fa - fc) > 1e-30 && std::abs(fb - fc) > 1e-30) {
            // 逆二次插值
            s = a * fb * fc / ((fa - fb) * (fa - fc)) +
                b * fa * fc / ((fb - fa) * (fb - fc)) +
                c * fa * fb / ((fc - fa) * (fc - fb));
        } else {
            // 割线法
            s = b - fb * (b - a) / (fb - fa);
        }
        
        // 检查是否需要使用二分法
        double cond1 = (s < (3*a + b)/4 || s > b);
        double cond2 = mflag && std::abs(s - b) >= std::abs(b - c) / 2;
        double cond3 = !mflag && std::abs(s - b) >= std::abs(c - d) / 2;
        double cond4 = mflag && std::abs(b - c) < tol;
        double cond5 = !mflag && std::abs(c - d) < tol;
        
        if (cond1 || cond2 || cond3 || cond4 || cond5) {
            s = (a + b) / 2;
            mflag = true;
        } else {
            mflag = false;
        }
        
        double fs = f(s);
        d = c;
        c = b;
        fc = fb;
        
        if (fa * fs < 0) {
            b = s;
            fb = fs;
        } else {
            a = s;
            fa = fs;
        }
        
        if (std::abs(fa) < std::abs(fb)) {
            std::swap(a, b);
            std::swap(fa, fb);
        }
    }
    
    result.root = b;
    result.residual = std::abs(fb);
    return result;
}

// ==================== 多项式求根 ====================

/**
 * @brief 多项式求根 (伴随矩阵法)
 * 
 * 对于多项式 p(x) = c[0] + c[1]*x + c[2]*x^2 + ... + c[n]*x^n
 * 通过计算伴随矩阵的特征值获得所有根
 * 
 * @param coeffs 系数向量 (升幂排列: c[0], c[1], ..., c[n])
 * @return 所有根 (可能是复数)
 */
inline std::vector<std::complex<double>> polynomial_roots(const std::vector<double>& coeffs) {
    std::vector<std::complex<double>> roots;
    
    // 去除前导零
    size_t n = coeffs.size();
    while (n > 0 && std::abs(coeffs[n-1]) < 1e-30) {
        --n;
    }
    
    if (n <= 1) {
        return roots;  // 常数或空多项式
    }
    
    if (n == 2) {
        // 线性: c[0] + c[1]*x = 0 => x = -c[0]/c[1]
        roots.push_back(-coeffs[0] / coeffs[1]);
        return roots;
    }
    
    if (n == 3) {
        // 二次方程
        double a = coeffs[2], b = coeffs[1], c = coeffs[0];
        double disc = b*b - 4*a*c;
        if (disc >= 0) {
            double sq = std::sqrt(disc);
            roots.push_back((-b + sq) / (2*a));
            roots.push_back((-b - sq) / (2*a));
        } else {
            double sq = std::sqrt(-disc);
            roots.push_back(std::complex<double>(-b/(2*a), sq/(2*a)));
            roots.push_back(std::complex<double>(-b/(2*a), -sq/(2*a)));
        }
        return roots;
    }
    
    // 构造伴随矩阵
    // 对于首一多项式 x^n + a_{n-1}*x^{n-1} + ... + a_0 = 0
    // 伴随矩阵:
    // [0   0   ...  0   -a_0   ]
    // [1   0   ...  0   -a_1   ]
    // [0   1   ...  0   -a_2   ]
    // [...                     ]
    // [0   0   ...  1   -a_{n-1}]
    
    int deg = static_cast<int>(n) - 1;
    double leading = coeffs[deg];
    
    MatrixX companion = MatrixX::Zero(deg, deg);
    for (int i = 0; i < deg - 1; ++i) {
        companion(i + 1, i) = 1.0;
    }
    for (int i = 0; i < deg; ++i) {
        companion(i, deg - 1) = -coeffs[i] / leading;
    }
    
    // 计算特征值
    Eigen::EigenSolver<MatrixX> es(companion, false);
    auto eigenvalues = es.eigenvalues();
    
    for (int i = 0; i < eigenvalues.size(); ++i) {
        roots.push_back(eigenvalues(i));
    }
    
    return roots;
}

/**
 * @brief 获取多项式的实根
 * @param coeffs 系数向量 (升幂排列)
 * @param tol 虚部容差
 */
inline std::vector<double> polynomial_real_roots(const std::vector<double>& coeffs,
                                                  double tol = 1e-10) {
    auto all_roots = polynomial_roots(coeffs);
    std::vector<double> real_roots;
    
    for (const auto& r : all_roots) {
        if (std::abs(r.imag()) < tol) {
            real_roots.push_back(r.real());
        }
    }
    
    return real_roots;
}

/**
 * @brief 多项式求值 (Horner 方法)
 * @param coeffs 系数 (升幂: c0 + c1*x + c2*x^2 + ...)
 * @param x 求值点
 */
inline double polynomial_eval(const std::vector<double>& coeffs, double x) {
    if (coeffs.empty()) return 0.0;
    
    double result = coeffs.back();
    for (int i = static_cast<int>(coeffs.size()) - 2; i >= 0; --i) {
        result = result * x + coeffs[i];
    }
    return result;
}

/**
 * @brief 多项式求值 (复数版本)
 */
inline std::complex<double> polynomial_eval(const std::vector<double>& coeffs, 
                                             std::complex<double> x) {
    if (coeffs.empty()) return 0.0;
    
    std::complex<double> result = coeffs.back();
    for (int i = static_cast<int>(coeffs.size()) - 2; i >= 0; --i) {
        result = result * x + coeffs[i];
    }
    return result;
}

} // namespace gnc::math
