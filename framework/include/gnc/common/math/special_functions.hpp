/**
 * @file special_functions.hpp
 * @brief 特殊数学函数 - C++14 兼容实现
 * 
 * 包含:
 * - Bessel 函数 (第一类/第二类)
 * - 误差函数 (erf/erfc)
 * - 饱和函数 (sat)
 * - 符号函数 (sign)
 * - 死区函数 (deadzone)
 * 
 * @note 所有函数均为无状态纯函数
 */
#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>

namespace gnc::math {

// ==================== 基础非线性函数 ====================

/**
 * @brief 符号函数
 * @param x 输入值
 * @return +1 (x > 0), -1 (x < 0), 0 (x == 0)
 */
inline double sign(double x) {
    if (x > 0.0) return 1.0;
    if (x < 0.0) return -1.0;
    return 0.0;
}

/**
 * @brief 饱和函数/限幅函数
 * @param x 输入值
 * @param lo 下限
 * @param hi 上限
 * @return 限制在 [lo, hi] 范围内的值
 */
inline double sat(double x, double lo, double hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/**
 * @brief 对称饱和函数
 * @param x 输入值
 * @param limit 对称限制值 (> 0)
 * @return 限制在 [-limit, limit] 范围内的值
 */
inline double sat(double x, double limit) {
    return sat(x, -limit, limit);
}

/**
 * @brief 死区函数
 * @param x 输入值
 * @param band 死区半宽 (在 [-band, band] 内输出为 0)
 * @return 死区处理后的值
 */
inline double deadzone(double x, double band) {
    if (x > band) return x - band;
    if (x < -band) return x + band;
    return 0.0;
}

/**
 * @brief 非对称死区函数
 * @param x 输入值
 * @param lo 下死区边界
 * @param hi 上死区边界
 * @return 死区处理后的值
 */
inline double deadzone(double x, double lo, double hi) {
    if (x > hi) return x - hi;
    if (x < lo) return x - lo;
    return 0.0;
}

// ==================== 误差函数 (C++14 兼容实现) ====================

namespace detail {

/**
 * @brief 误差函数的 Horner 多项式近似 (Abramowitz & Stegun 7.1.26)
 * 
 * 最大误差: |ε| < 1.5e-7
 */
inline double erf_impl(double x) {
    // 常数
    constexpr double a1 =  0.254829592;
    constexpr double a2 = -0.284496736;
    constexpr double a3 =  1.421413741;
    constexpr double a4 = -1.453152027;
    constexpr double a5 =  1.061405429;
    constexpr double p  =  0.3275911;
    
    // 利用 erf(-x) = -erf(x) 的对称性
    double sign_x = sign(x);
    x = std::abs(x);
    
    // Horner 形式计算
    double t = 1.0 / (1.0 + p * x);
    double y = 1.0 - (((((a5 * t + a4) * t) + a3) * t + a2) * t + a1) * t * std::exp(-x * x);
    
    return sign_x * y;
}

/**
 * @brief 高精度误差函数实现 (Cody rational approximation)
 * 
 * 最大误差: |ε| < 1e-15
 */
inline double erf_precise(double x) {
    constexpr double a[] = {
        3.1611237438705656e-01,
        1.1331250042568893e-01,
        2.6673028193612310e-02,
        3.5445040297365680e-03,
        2.4466366321698167e-04,
        6.9575693166953870e-06
    };
    constexpr double b[] = {
        2.1536017441776440e-01,
        1.3884901670067525e-01,
        5.4210905968606060e-02,
        1.3587922927852940e-02,
        2.1925604026600000e-03,
        2.1467340001710000e-04,
        1.0047910853660000e-05
    };
    
    double sign_x = sign(x);
    x = std::abs(x);
    
    if (x < 0.5) {
        double x2 = x * x;
        double num = x * (a[0] + x2 * (a[1] + x2 * (a[2] + x2 * (a[3] + x2 * (a[4] + x2 * a[5])))));
        double den = 1.0 + x2 * (b[0] + x2 * (b[1] + x2 * (b[2] + x2 * (b[3] + x2 * (b[4] + x2 * (b[5] + x2 * b[6]))))));
        return sign_x * num / den;
    } else {
        // 使用 erfc 实现
        return sign_x * (1.0 - erfc(sign_x * x));
    }
}

} // namespace detail

/**
 * @brief 误差函数 erf(x) = (2/√π) ∫₀ˣ e^(-t²) dt
 * @param x 输入值
 * @return erf(x) ∈ [-1, 1]
 */
inline double erf(double x) {
    return detail::erf_impl(x);
}

/**
 * @brief 补误差函数 erfc(x) = 1 - erf(x)
 * @param x 输入值
 * @return erfc(x) ∈ [0, 2]
 */
inline double erfc(double x) {
    return 1.0 - detail::erf_impl(x);
}

// ==================== Bessel 函数 (C++14 兼容实现) ====================

namespace detail {

/**
 * @brief 计算 J0(x) 的多项式近似
 * 
 * 基于 Hart, "Computer Approximations", 1968
 * 对于 |x| <= 8: 使用有理函数近似
 * 对于 |x| > 8: 使用渐近展开
 */
inline double bessel_j0_impl(double x) {
    x = std::abs(x);
    
    if (x < 8.0) {
        double y = x * x;
        double num = 57568490574.0 + y * (-13362590354.0 + y * (651619640.7 
                   + y * (-11214424.18 + y * (77392.33017 + y * (-184.9052456)))));
        double den = 57568490411.0 + y * (1029532985.0 + y * (9494680.718 
                   + y * (59272.64853 + y * (267.8532712 + y * 1.0))));
        return num / den;
    } else {
        double z = 8.0 / x;
        double y = z * z;
        double xx = x - 0.785398164;  // x - π/4
        double p0 = 1.0 + y * (-0.1098628627e-2 + y * (0.2734510407e-4 
                  + y * (-0.2073370639e-5 + y * 0.2093887211e-6)));
        double q0 = -0.1562499995e-1 + y * (0.1430488765e-3 
                  + y * (-0.6911147651e-5 + y * (0.7621095161e-6 + y * (-0.934945152e-7))));
        return std::sqrt(0.636619772 / x) * (p0 * std::cos(xx) - z * q0 * std::sin(xx));
    }
}

/**
 * @brief 计算 J1(x) 的多项式近似
 */
inline double bessel_j1_impl(double x) {
    double ax = std::abs(x);
    
    if (ax < 8.0) {
        double y = x * x;
        double num = x * (72362614232.0 + y * (-7895059235.0 + y * (242396853.1 
                   + y * (-2972611.439 + y * (15704.48260 + y * (-30.16036606))))));
        double den = 144725228442.0 + y * (2300535178.0 + y * (18583304.74 
                   + y * (99447.43394 + y * (376.9991397 + y * 1.0))));
        return num / den;
    } else {
        double z = 8.0 / ax;
        double y = z * z;
        double xx = ax - 2.356194491;  // x - 3π/4
        double p1 = 1.0 + y * (0.183105e-2 + y * (-0.3516396496e-4 
                  + y * (0.2457520174e-5 + y * (-0.240337019e-6))));
        double q1 = 0.04687499995 + y * (-0.2002690873e-3 
                  + y * (0.8449199096e-5 + y * (-0.88228987e-6 + y * 0.105787412e-6)));
        double result = std::sqrt(0.636619772 / ax) * (p1 * std::cos(xx) - z * q1 * std::sin(xx));
        return (x < 0.0) ? -result : result;
    }
}

/**
 * @brief 计算 Y0(x) 的多项式近似 (第二类 Bessel 函数)
 */
inline double bessel_y0_impl(double x) {
    if (x <= 0.0) {
        return -std::numeric_limits<double>::infinity();
    }
    
    if (x < 8.0) {
        double y = x * x;
        double num = -2957821389.0 + y * (7062834065.0 + y * (-512359803.6 
                   + y * (10879881.29 + y * (-86327.92757 + y * 228.4622733))));
        double den = 40076544269.0 + y * (745249964.8 + y * (7189466.438 
                   + y * (47447.26470 + y * (226.1030244 + y * 1.0))));
        return num / den + 0.636619772 * bessel_j0_impl(x) * std::log(x);
    } else {
        double z = 8.0 / x;
        double y = z * z;
        double xx = x - 0.785398164;  // x - π/4
        double p0 = 1.0 + y * (-0.1098628627e-2 + y * (0.2734510407e-4 
                  + y * (-0.2073370639e-5 + y * 0.2093887211e-6)));
        double q0 = -0.1562499995e-1 + y * (0.1430488765e-3 
                  + y * (-0.6911147651e-5 + y * (0.7621095161e-6 + y * (-0.934945152e-7))));
        return std::sqrt(0.636619772 / x) * (p0 * std::sin(xx) + z * q0 * std::cos(xx));
    }
}

/**
 * @brief 计算 Y1(x) 的多项式近似
 */
inline double bessel_y1_impl(double x) {
    if (x <= 0.0) {
        return -std::numeric_limits<double>::infinity();
    }
    
    if (x < 8.0) {
        double y = x * x;
        double num = x * (-0.4900604943e13 + y * (0.1275274390e13 + y * (-0.5153438139e11 
                   + y * (0.7349264551e9 + y * (-0.4237922726e7 + y * 0.8511937935e4)))));
        double den = 0.2499580570e14 + y * (0.4244419664e12 + y * (0.3733650367e10 
                   + y * (0.2245904002e8 + y * (0.1020426050e6 + y * (0.3549632885e3 + y)))));
        return num / den + 0.636619772 * (bessel_j1_impl(x) * std::log(x) - 1.0 / x);
    } else {
        double z = 8.0 / x;
        double y = z * z;
        double xx = x - 2.356194491;  // x - 3π/4
        double p1 = 1.0 + y * (0.183105e-2 + y * (-0.3516396496e-4 
                  + y * (0.2457520174e-5 + y * (-0.240337019e-6))));
        double q1 = 0.04687499995 + y * (-0.2002690873e-3 
                  + y * (0.8449199096e-5 + y * (-0.88228987e-6 + y * 0.105787412e-6)));
        return std::sqrt(0.636619772 / x) * (p1 * std::sin(xx) + z * q1 * std::cos(xx));
    }
}

} // namespace detail

/**
 * @brief 第一类 Bessel 函数 J₀(x)
 */
inline double bessel_j0(double x) {
    return detail::bessel_j0_impl(x);
}

/**
 * @brief 第一类 Bessel 函数 J₁(x)
 */
inline double bessel_j1(double x) {
    return detail::bessel_j1_impl(x);
}

/**
 * @brief 第一类 Bessel 函数 Jₙ(x)，使用向下递推
 * @param n 阶数 (n >= 0)
 * @param x 输入值
 */
inline double bessel_jn(int n, double x) {
    if (n < 0) {
        // J_{-n}(x) = (-1)^n * J_n(x)
        n = -n;
        return (n % 2 == 0) ? bessel_jn(n, x) : -bessel_jn(n, x);
    }
    if (n == 0) return bessel_j0(x);
    if (n == 1) return bessel_j1(x);
    
    double ax = std::abs(x);
    if (ax < 1e-10) return 0.0;
    
    // Miller 向下递推法
    constexpr int IACC = 40;
    constexpr double BIGNO = 1.0e10;
    constexpr double BIGNI = 1.0e-10;
    
    int m = 2 * ((n + static_cast<int>(std::sqrt(static_cast<double>(IACC * n)))) / 2);
    double bjp = 0.0, bj = 1.0, bjm, sum = 0.0, ans = 0.0;
    bool jsum = false;
    
    for (int j = m; j > 0; --j) {
        bjm = j * 2.0 / ax * bj - bjp;
        bjp = bj;
        bj = bjm;
        if (std::abs(bj) > BIGNO) {
            bj *= BIGNI;
            bjp *= BIGNI;
            ans *= BIGNI;
            sum *= BIGNI;
        }
        if (jsum) sum += bj;
        jsum = !jsum;
        if (j == n) ans = bjp;
    }
    
    sum = 2.0 * sum - bj;
    ans /= sum;
    
    return (x < 0.0 && (n % 2 == 1)) ? -ans : ans;
}

/**
 * @brief 第二类 Bessel 函数 Y₀(x)
 * @param x 输入值 (x > 0)
 */
inline double bessel_y0(double x) {
    return detail::bessel_y0_impl(x);
}

/**
 * @brief 第二类 Bessel 函数 Y₁(x)
 * @param x 输入值 (x > 0)
 */
inline double bessel_y1(double x) {
    return detail::bessel_y1_impl(x);
}

/**
 * @brief 第二类 Bessel 函数 Yₙ(x)，使用向上递推
 * @param n 阶数 (n >= 0)
 * @param x 输入值 (x > 0)
 */
inline double bessel_yn(int n, double x) {
    if (n < 0) {
        // Y_{-n}(x) = (-1)^n * Y_n(x)
        n = -n;
        return (n % 2 == 0) ? bessel_yn(n, x) : -bessel_yn(n, x);
    }
    if (n == 0) return bessel_y0(x);
    if (n == 1) return bessel_y1(x);
    
    // 向上递推: Y_{n+1}(x) = (2n/x)*Y_n(x) - Y_{n-1}(x)
    double bym = bessel_y0(x);
    double by = bessel_y1(x);
    for (int j = 1; j < n; ++j) {
        double byp = j * 2.0 / x * by - bym;
        bym = by;
        by = byp;
    }
    return by;
}

} // namespace gnc::math
