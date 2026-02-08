/**
 * @file statistics.hpp
 * @brief 概率统计 - 随机数生成、分布和协方差运算
 * 
 * 包含:
 * - 线程安全随机数生成器
 * - 高斯分布 (PDF/CDF/采样)
 * - 协方差传播
 * - 样本统计
 * 
 * @note RandomGenerator 是有状态类，其他为无状态函数
 */
#pragma once

#include "eigen_types.hpp"
#include "special_functions.hpp"
#include <random>
#include <cmath>
#include <mutex>

namespace gnc::math {

// ==================== 高斯分布函数 ====================

/**
 * @brief 高斯概率密度函数
 * @param x 采样点
 * @param mu 均值
 * @param sigma 标准差
 */
inline double gaussian_pdf(double x, double mu = 0.0, double sigma = 1.0) {
    double z = (x - mu) / sigma;
    return std::exp(-0.5 * z * z) / (sigma * std::sqrt(2.0 * constants::PI));
}

/**
 * @brief 高斯累积分布函数
 * @param x 采样点
 * @param mu 均值
 * @param sigma 标准差
 */
inline double gaussian_cdf(double x, double mu = 0.0, double sigma = 1.0) {
    return 0.5 * (1.0 + erf((x - mu) / (sigma * std::sqrt(2.0))));
}

/**
 * @brief 高斯分布分位数 (逆 CDF)
 * 
 * 使用 Rational Approximation
 */
inline double gaussian_quantile(double p, double mu = 0.0, double sigma = 1.0) {
    // Abramowitz & Stegun 近似
    if (p <= 0.0) return -std::numeric_limits<double>::infinity();
    if (p >= 1.0) return std::numeric_limits<double>::infinity();
    
    // 对称性
    bool lower = (p < 0.5);
    double pp = lower ? p : 1.0 - p;
    
    double t = std::sqrt(-2.0 * std::log(pp));
    constexpr double c0 = 2.515517;
    constexpr double c1 = 0.802853;
    constexpr double c2 = 0.010328;
    constexpr double d1 = 1.432788;
    constexpr double d2 = 0.189269;
    constexpr double d3 = 0.001308;
    
    double z = t - (c0 + c1*t + c2*t*t) / (1.0 + d1*t + d2*t*t + d3*t*t*t);
    
    return mu + sigma * (lower ? -z : z);
}

// ==================== 随机数生成器 ====================

/**
 * @brief 线程安全随机数生成器
 */
class RandomGenerator {
public:
    /**
     * @brief 构造函数
     * @param seed 随机种子 (默认使用 random_device)
     */
    explicit RandomGenerator(unsigned int seed = 0) {
        if (seed == 0) {
            std::random_device rd;
            seed = rd();
        }
        rng_.seed(seed);
    }
    
    /**
     * @brief 设置种子
     */
    void seed(unsigned int s) {
        std::lock_guard<std::mutex> lock(mutex_);
        rng_.seed(s);
    }
    
    /**
     * @brief 均匀分布 [0, 1)
     */
    double uniform() {
        std::lock_guard<std::mutex> lock(mutex_);
        return uniform_dist_(rng_);
    }
    
    /**
     * @brief 均匀分布 [lo, hi)
     */
    double uniform(double lo, double hi) {
        return lo + (hi - lo) * uniform();
    }
    
    /**
     * @brief 标准正态分布 N(0, 1)
     */
    double randn() {
        std::lock_guard<std::mutex> lock(mutex_);
        return normal_dist_(rng_);
    }
    
    /**
     * @brief 正态分布 N(mu, sigma^2)
     */
    double gaussian(double mu, double sigma) {
        return mu + sigma * randn();
    }
    
    /**
     * @brief 多维正态分布采样
     * @param mean 均值向量
     * @param cov 协方差矩阵
     * @return 采样向量
     */
    VectorX multivariate_gaussian(const VectorX& mean, const MatrixX& cov) {
        int n = mean.size();
        
        // Cholesky 分解: cov = L * L^T
        Eigen::LLT<MatrixX> llt(cov);
        if (llt.info() != Eigen::Success) {
            throw std::runtime_error("Covariance matrix is not positive definite");
        }
        MatrixX L = llt.matrixL();
        
        // 生成独立标准正态
        VectorX z(n);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (int i = 0; i < n; ++i) {
                z(i) = normal_dist_(rng_);
            }
        }
        
        return mean + L * z;
    }
    
    /**
     * @brief 整数均匀分布 [lo, hi]
     */
    int randint(int lo, int hi) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::uniform_int_distribution<int> dist(lo, hi);
        return dist(rng_);
    }
    
    /**
     * @brief 向量正态采样
     */
    VectorX randn_vector(int n) {
        VectorX v(n);
        std::lock_guard<std::mutex> lock(mutex_);
        for (int i = 0; i < n; ++i) {
            v(i) = normal_dist_(rng_);
        }
        return v;
    }
    
private:
    std::mt19937_64 rng_;
    std::uniform_real_distribution<double> uniform_dist_{0.0, 1.0};
    std::normal_distribution<double> normal_dist_{0.0, 1.0};
    std::mutex mutex_;
};

// ==================== 协方差运算 ====================

/**
 * @brief 协方差传播 (线性系统)
 * 
 * 对于 x_{k+1} = F * x_k + w, w ~ N(0, Q)
 * 协方差传播: P_{k+1} = F * P_k * F^T + Q
 * 
 * @param F 状态转移矩阵
 * @param P 当前协方差
 * @param Q 过程噪声协方差
 * @return 传播后的协方差
 */
inline MatrixX covariance_propagate(const MatrixX& F, const MatrixX& P, const MatrixX& Q) {
    return F * P * F.transpose() + Q;
}

/**
 * @brief Joseph 形式协方差更新 (数值稳定)
 * 
 * P = (I - K*H) * P * (I - K*H)^T + K * R * K^T
 * 
 * @param P 先验协方差
 * @param K Kalman 增益
 * @param H 观测矩阵
 * @param R 观测噪声协方差
 */
inline MatrixX covariance_update_joseph(const MatrixX& P, const MatrixX& K, 
                                        const MatrixX& H, const MatrixX& R) {
    MatrixX I_KH = MatrixX::Identity(P.rows(), P.cols()) - K * H;
    return I_KH * P * I_KH.transpose() + K * R * K.transpose();
}

/**
 * @brief 标准协方差更新
 * 
 * P = (I - K*H) * P
 */
inline MatrixX covariance_update(const MatrixX& P, const MatrixX& K, const MatrixX& H) {
    int n = P.rows();
    return (MatrixX::Identity(n, n) - K * H) * P;
}

// ==================== 样本统计 ====================

/**
 * @brief 样本均值
 */
inline double sample_mean(const std::vector<double>& data) {
    if (data.empty()) return 0.0;
    double sum = 0.0;
    for (double x : data) sum += x;
    return sum / static_cast<double>(data.size());
}

/**
 * @brief 样本方差 (无偏)
 */
inline double sample_variance(const std::vector<double>& data) {
    if (data.size() < 2) return 0.0;
    double mu = sample_mean(data);
    double sum_sq = 0.0;
    for (double x : data) {
        double d = x - mu;
        sum_sq += d * d;
    }
    return sum_sq / static_cast<double>(data.size() - 1);
}

/**
 * @brief 样本标准差
 */
inline double sample_stddev(const std::vector<double>& data) {
    return std::sqrt(sample_variance(data));
}

/**
 * @brief 向量样本均值
 */
inline VectorX sample_mean(const std::vector<VectorX>& data) {
    if (data.empty()) return VectorX();
    
    VectorX sum = VectorX::Zero(data[0].size());
    for (const auto& x : data) {
        sum += x;
    }
    return sum / static_cast<double>(data.size());
}

/**
 * @brief 样本协方差矩阵 (无偏)
 */
inline MatrixX sample_covariance(const std::vector<VectorX>& data) {
    if (data.size() < 2) return MatrixX();
    
    VectorX mu = sample_mean(data);
    int n = mu.size();
    MatrixX cov = MatrixX::Zero(n, n);
    
    for (const auto& x : data) {
        VectorX d = x - mu;
        cov += d * d.transpose();
    }
    
    return cov / static_cast<double>(data.size() - 1);
}

/**
 * @brief 加权均值
 */
inline double weighted_mean(const std::vector<double>& data, 
                            const std::vector<double>& weights) {
    if (data.size() != weights.size() || data.empty()) return 0.0;
    
    double sum = 0.0, w_sum = 0.0;
    for (size_t i = 0; i < data.size(); ++i) {
        sum += weights[i] * data[i];
        w_sum += weights[i];
    }
    return sum / w_sum;
}

/**
 * @brief 加权协方差
 */
inline MatrixX weighted_covariance(const std::vector<VectorX>& data,
                                    const std::vector<double>& weights) {
    if (data.size() != weights.size() || data.size() < 2) return MatrixX();
    
    int n = data[0].size();
    
    // 计算加权均值
    VectorX mu = VectorX::Zero(n);
    double w_sum = 0.0;
    for (size_t i = 0; i < data.size(); ++i) {
        mu += weights[i] * data[i];
        w_sum += weights[i];
    }
    mu /= w_sum;
    
    // 计算加权协方差
    MatrixX cov = MatrixX::Zero(n, n);
    for (size_t i = 0; i < data.size(); ++i) {
        VectorX d = data[i] - mu;
        cov += weights[i] * d * d.transpose();
    }
    
    return cov / w_sum;
}

} // namespace gnc::math
