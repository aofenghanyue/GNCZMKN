/**
 * @file moving_average.hpp
 * @brief 移动平均滤波器
 */
#pragma once

#include "filter_base.hpp"
#include <vector>
#include <numeric>

namespace gnc::libraries::filters {

/**
 * @brief 简单移动平均滤波器 (SMA)
 */
class MovingAverageFilter : public IFilter {
public:
    /**
     * @brief 构造
     * @param window_size 窗口大小
     */
    explicit MovingAverageFilter(int window_size)
        : buffer_(window_size, 0.0), size_(window_size), index_(0), sum_(0.0) {}
    
    /**
     * @brief 滤波
     */
    double filter(double input) override {
        // 减去最旧的值，加入新值
        sum_ -= buffer_[index_];
        buffer_[index_] = input;
        sum_ += input;
        
        index_ = (index_ + 1) % size_;
        count_ = std::min(count_ + 1, size_);
        
        return sum_ / count_;
    }
    
    /**
     * @brief 重置
     */
    void reset() override {
        std::fill(buffer_.begin(), buffer_.end(), 0.0);
        index_ = 0;
        sum_ = 0.0;
        count_ = 0;
    }
    
private:
    std::vector<double> buffer_;
    int size_;
    int index_;
    double sum_;
    int count_ = 0;
};

/**
 * @brief 指数移动平均滤波器 (EMA)
 * 
 * y[n] = α * x[n] + (1-α) * y[n-1]
 */
class ExponentialMovingAverage : public IFilter {
public:
    /**
     * @brief 从平滑因子构造
     * @param alpha 平滑因子 (0 < α < 1, 越大响应越快)
     */
    explicit ExponentialMovingAverage(double alpha)
        : alpha_(alpha), y_(0.0), initialized_(false) {}
    
    /**
     * @brief 从等效窗口大小构造
     * α ≈ 2 / (N + 1)
     */
    static ExponentialMovingAverage fromWindowSize(int N) {
        return ExponentialMovingAverage(2.0 / (N + 1));
    }
    
    /**
     * @brief 从时间常数构造
     * α = 1 - exp(-dt/τ) ≈ dt/τ (当 dt << τ)
     */
    static ExponentialMovingAverage fromTimeConstant(double tau, double dt) {
        return ExponentialMovingAverage(1.0 - std::exp(-dt / tau));
    }
    
    double filter(double input) override {
        if (!initialized_) {
            y_ = input;
            initialized_ = true;
        } else {
            y_ = alpha_ * input + (1.0 - alpha_) * y_;
        }
        return y_;
    }
    
    void reset() override {
        y_ = 0.0;
        initialized_ = false;
    }
    
    void setAlpha(double alpha) { alpha_ = alpha; }
    double getAlpha() const { return alpha_; }
    
private:
    double alpha_;
    double y_;
    bool initialized_;
};

/**
 * @brief 加权移动平均滤波器 (WMA)
 * 
 * 线性加权：最新样本权重最高
 */
class WeightedMovingAverage : public IFilter {
public:
    explicit WeightedMovingAverage(int window_size)
        : buffer_(window_size, 0.0), size_(window_size), index_(0) {
        // 计算权重和
        weight_sum_ = (size_ * (size_ + 1)) / 2.0;
    }
    
    double filter(double input) override {
        buffer_[index_] = input;
        index_ = (index_ + 1) % size_;
        count_ = std::min(count_ + 1, size_);
        
        if (count_ < size_) {
            // 还未填满，使用简单平均
            double sum = 0;
            for (int i = 0; i < count_; ++i) sum += buffer_[i];
            return sum / count_;
        }
        
        // 加权平均
        double sum = 0;
        int weight = 1;
        for (int i = 0; i < size_; ++i) {
            int buf_idx = (index_ + i) % size_;
            sum += weight * buffer_[buf_idx];
            ++weight;
        }
        return sum / weight_sum_;
    }
    
    void reset() override {
        std::fill(buffer_.begin(), buffer_.end(), 0.0);
        index_ = 0;
        count_ = 0;
    }
    
private:
    std::vector<double> buffer_;
    int size_;
    int index_;
    int count_ = 0;
    double weight_sum_;
};

/**
 * @brief 中值滤波器
 * 
 * 对脉冲噪声有较好抑制效果
 */
class MedianFilter : public IFilter {
public:
    explicit MedianFilter(int window_size)
        : buffer_(window_size, 0.0), size_(window_size), index_(0) {}
    
    double filter(double input) override {
        buffer_[index_] = input;
        index_ = (index_ + 1) % size_;
        count_ = std::min(count_ + 1, size_);
        
        // 排序并取中值
        std::vector<double> sorted(buffer_.begin(), buffer_.begin() + count_);
        std::sort(sorted.begin(), sorted.end());
        
        if (count_ % 2 == 0) {
            return (sorted[count_/2 - 1] + sorted[count_/2]) / 2.0;
        } else {
            return sorted[count_/2];
        }
    }
    
    void reset() override {
        std::fill(buffer_.begin(), buffer_.end(), 0.0);
        index_ = 0;
        count_ = 0;
    }
    
private:
    std::vector<double> buffer_;
    int size_;
    int index_;
    int count_ = 0;
};

} // namespace gnc::libraries::filters
