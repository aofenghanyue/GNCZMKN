/**
 * @file notch.hpp
 * @brief 陷波器
 * 
 * 用于消除特定频率的干扰，如电源频率
 */
#pragma once

#include "filter_base.hpp"
#include <cmath>

namespace gnc::libraries::filters {

/**
 * @brief 陷波器 (Notch Filter)
 * 
 * 二阶 IIR 陷波器，在指定频率处形成零点
 */
class NotchFilter : public IFilter {
public:
    /**
     * @brief 构造陷波器
     * @param notch_freq_hz 陷波频率 (Hz)
     * @param Q Q 因子 (越大带宽越窄)
     * @param sample_hz 采样频率 (Hz)
     */
    NotchFilter(double notch_freq_hz, double Q, double sample_hz) {
        double w0 = 2.0 * M_PI * notch_freq_hz / sample_hz;
        double cos_w0 = std::cos(w0);
        double alpha = std::sin(w0) / (2.0 * Q);
        
        // 归一化系数
        double norm = 1.0 / (1.0 + alpha);
        
        b0_ = norm;
        b1_ = -2.0 * cos_w0 * norm;
        b2_ = norm;
        a1_ = b1_;  // = -2*cos(w0) * norm
        a2_ = (1.0 - alpha) * norm;
    }
    
    /**
     * @brief 从带宽构造
     * @param notch_freq_hz 陷波频率
     * @param bandwidth_hz 3dB 带宽
     * @param sample_hz 采样频率
     */
    static NotchFilter fromBandwidth(double notch_freq_hz, double bandwidth_hz, 
                                      double sample_hz) {
        double Q = notch_freq_hz / bandwidth_hz;
        return NotchFilter(notch_freq_hz, Q, sample_hz);
    }
    
    /**
     * @brief 滤波
     */
    double filter(double input) override {
        // Direct Form II Transposed
        double output = b0_ * input + z1_;
        z1_ = b1_ * input - a1_ * output + z2_;
        z2_ = b2_ * input - a2_ * output;
        return output;
    }
    
    /**
     * @brief 重置
     */
    void reset() override {
        z1_ = z2_ = 0;
    }
    
private:
    double b0_, b1_, b2_;
    double a1_, a2_;
    double z1_ = 0, z2_ = 0;
};

/**
 * @brief 峰值滤波器 (Peak/Bell Filter)
 * 
 * 与陷波器相反，在特定频率处增益
 */
class PeakFilter : public IFilter {
public:
    /**
     * @brief 构造
     * @param freq_hz 中心频率
     * @param Q Q 因子
     * @param gain_db 增益 (dB)
     * @param sample_hz 采样频率
     */
    PeakFilter(double freq_hz, double Q, double gain_db, double sample_hz) {
        double w0 = 2.0 * M_PI * freq_hz / sample_hz;
        double cos_w0 = std::cos(w0);
        double sin_w0 = std::sin(w0);
        double A = std::pow(10.0, gain_db / 40.0);  // sqrt(10^(dB/20))
        double alpha = sin_w0 / (2.0 * Q);
        
        double norm = 1.0 / (1.0 + alpha / A);
        
        b0_ = (1.0 + alpha * A) * norm;
        b1_ = -2.0 * cos_w0 * norm;
        b2_ = (1.0 - alpha * A) * norm;
        a1_ = b1_;
        a2_ = (1.0 - alpha / A) * norm;
    }
    
    double filter(double input) override {
        double output = b0_ * input + z1_;
        z1_ = b1_ * input - a1_ * output + z2_;
        z2_ = b2_ * input - a2_ * output;
        return output;
    }
    
    void reset() override { z1_ = z2_ = 0; }
    
private:
    double b0_, b1_, b2_;
    double a1_, a2_;
    double z1_ = 0, z2_ = 0;
};

} // namespace gnc::libraries::filters
