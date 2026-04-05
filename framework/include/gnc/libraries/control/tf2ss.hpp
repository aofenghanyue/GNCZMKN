/**
 * @file tf2ss.hpp
 * @brief 传递函数转状态空间
 * 
 * 将传递函数 H(s) = num(s)/den(s) 转换为可控标准型状态空间
 */
#pragma once

#include "state_space.hpp"
#include <vector>
#include <algorithm>

namespace gnc::libraries::control {

/**
 * @brief 传递函数表示
 */
struct TransferFunction {
    std::vector<double> num;  ///< 分子系数 (升幂: b0 + b1*s + b2*s^2 + ...)
    std::vector<double> den;  ///< 分母系数 (升幂: a0 + a1*s + a2*s^2 + ...)
    
    /**
     * @brief 构造传递函数
     * @param numerator 分子系数 (升幂)
     * @param denominator 分母系数 (升幂)
     */
    TransferFunction(std::vector<double> numerator, std::vector<double> denominator)
        : num(std::move(numerator)), den(std::move(denominator)) {}
    
    /**
     * @brief 传递函数阶数
     */
    int order() const { 
        return static_cast<int>(den.size()) - 1; 
    }
    
    /**
     * @brief 归一化 (使最高次项系数为 1)
     */
    void normalize() {
        if (den.empty()) return;
        double a_n = den.back();
        if (std::abs(a_n) < 1e-30) return;
        
        for (auto& c : num) c /= a_n;
        for (auto& c : den) c /= a_n;
    }
};

/**
 * @brief 传递函数转状态空间 (可控标准型)
 * 
 * 对于传递函数:
 * H(s) = (b_m*s^m + ... + b_1*s + b_0) / (s^n + a_{n-1}*s^{n-1} + ... + a_0)
 * 
 * 生成可控标准型:
 * A = [0      1      0    ...  0    ]
 *     [0      0      1    ...  0    ]
 *     [...                          ]
 *     [-a_0  -a_1  -a_2  ... -a_{n-1}]
 * 
 * B = [0; 0; ...; 1]
 * C = [b_0-a_0*b_n, b_1-a_1*b_n, ..., b_{n-1}-a_{n-1}*b_n]
 * D = [b_n]
 * 
 * @param tf 传递函数
 * @return 状态空间模型
 */
inline StateSpaceModel tf2ss(TransferFunction tf) {
    tf.normalize();
    
    int n = tf.order();
    if (n <= 0) {
        // 常数传递函数
        return StateSpaceModel(
            MatrixX::Zero(1, 1),
            MatrixX::Zero(1, 1),
            MatrixX::Zero(1, 1),
            (MatrixX(1, 1) << (tf.num.empty() ? 0.0 : tf.num[0])).finished()
        );
    }
    
    // 扩展分子系数到与分母相同长度
    std::vector<double> b(n + 1, 0.0);
    for (size_t i = 0; i < tf.num.size() && i <= static_cast<size_t>(n); ++i) {
        b[i] = tf.num[i];
    }
    
    // 构造 A (伴随矩阵形式)
    MatrixX A = MatrixX::Zero(n, n);
    for (int i = 0; i < n - 1; ++i) {
        A(i, i + 1) = 1.0;
    }
    for (int i = 0; i < n; ++i) {
        A(n - 1, i) = -tf.den[i];  // 最后一行是 -[a_0, a_1, ..., a_{n-1}]
    }
    
    // 构造 B
    MatrixX B = MatrixX::Zero(n, 1);
    B(n - 1, 0) = 1.0;
    
    // 构造 C 和 D
    double b_n = b[n];  // 分子最高次系数
    MatrixX C = MatrixX::Zero(1, n);
    for (int i = 0; i < n; ++i) {
        C(0, i) = b[i] - tf.den[i] * b_n;
    }
    
    MatrixX D(1, 1);
    D(0, 0) = b_n;
    
    return StateSpaceModel(A, B, C, D, SystemType::Continuous);
}

/**
 * @brief 从 s 域极点和零点构造传递函数
 * @param zeros 零点列表
 * @param poles 极点列表
 * @param gain 增益
 */
inline TransferFunction zpk2tf(const std::vector<std::complex<double>>& zeros,
                                const std::vector<std::complex<double>>& poles,
                                double gain = 1.0) {
    // 从极点构造分母
    std::vector<double> den = {1.0};
    for (const auto& p : poles) {
        // 乘以 (s - p)
        std::vector<double> factor;
        if (std::abs(p.imag()) < 1e-10) {
            factor = {-p.real(), 1.0};
        } else {
            // 复共轭对: (s - p)(s - p*) = s^2 - 2*Re(p)*s + |p|^2
            double re = p.real();
            double mag2 = std::norm(p);
            factor = {mag2, -2*re, 1.0};
        }
        
        // 多项式乘法
        std::vector<double> new_den(den.size() + factor.size() - 1, 0.0);
        for (size_t i = 0; i < den.size(); ++i) {
            for (size_t j = 0; j < factor.size(); ++j) {
                new_den[i + j] += den[i] * factor[j];
            }
        }
        den = new_den;
    }
    
    // 从零点构造分子
    std::vector<double> num = {gain};
    for (const auto& z : zeros) {
        std::vector<double> factor;
        if (std::abs(z.imag()) < 1e-10) {
            factor = {-z.real(), 1.0};
        } else {
            double re = z.real();
            double mag2 = std::norm(z);
            factor = {mag2, -2*re, 1.0};
        }
        
        std::vector<double> new_num(num.size() + factor.size() - 1, 0.0);
        for (size_t i = 0; i < num.size(); ++i) {
            for (size_t j = 0; j < factor.size(); ++j) {
                new_num[i + j] += num[i] * factor[j];
            }
        }
        num = new_num;
    }
    
    return TransferFunction(num, den);
}

/**
 * @brief 常用传递函数构造
 */
namespace tf {

/// 一阶惯性环节 G(s) = K / (τs + 1)
inline TransferFunction first_order(double K, double tau) {
    return TransferFunction({K}, {1.0, tau});
}

/// 二阶系统 G(s) = K*ωn² / (s² + 2ζωn*s + ωn²)
inline TransferFunction second_order(double K, double wn, double zeta) {
    double wn2 = wn * wn;
    return TransferFunction({K * wn2}, {wn2, 2*zeta*wn, 1.0});
}

/// 积分器 G(s) = K/s
inline TransferFunction integrator(double K = 1.0) {
    return TransferFunction({K}, {0.0, 1.0});
}

/// 微分器 G(s) = K*s
inline TransferFunction differentiator(double K = 1.0) {
    return TransferFunction({0.0, K}, {1.0});
}

/// 超前校正 G(s) = (τ1*s + 1) / (τ2*s + 1)
inline TransferFunction lead_compensator(double tau1, double tau2) {
    return TransferFunction({1.0, tau1}, {1.0, tau2});
}

/// 滞后校正
inline TransferFunction lag_compensator(double tau1, double tau2) {
    return lead_compensator(tau1, tau2);
}

} // namespace tf

} // namespace gnc::libraries::control
