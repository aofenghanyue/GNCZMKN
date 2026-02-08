/**
 * @file butterworth.hpp
 * @brief Butterworth 数字滤波器
 * 
 * 特点: 最大平坦幅频响应
 */
#pragma once

#include "filter_base.hpp"
#include <vector>
#include <cmath>
#include <complex>

namespace gnc::libraries::filters {

/**
 * @brief Butterworth 滤波器 (Biquad 级联实现)
 */
class ButterworthFilter : public IFilter {
public:
    /**
     * @brief 构造低通滤波器
     * @param order 阶数 (1-8)
     * @param cutoff_hz 截止频率 (Hz)
     * @param sample_hz 采样频率 (Hz)
     */
    static ButterworthFilter lowpass(int order, double cutoff_hz, double sample_hz) {
        ButterworthFilter filt;
        filt.design(order, cutoff_hz, sample_hz, FilterType::Lowpass);
        return filt;
    }
    
    /**
     * @brief 构造高通滤波器
     */
    static ButterworthFilter highpass(int order, double cutoff_hz, double sample_hz) {
        ButterworthFilter filt;
        filt.design(order, cutoff_hz, sample_hz, FilterType::Highpass);
        return filt;
    }
    
    /**
     * @brief 构造带通滤波器
     * @param order 每边阶数 (总阶数 = 2*order)
     * @param lo_hz 下截止频率
     * @param hi_hz 上截止频率
     * @param sample_hz 采样频率
     */
    static ButterworthFilter bandpass(int order, double lo_hz, double hi_hz, double sample_hz) {
        ButterworthFilter filt;
        filt.designBandpass(order, lo_hz, hi_hz, sample_hz);
        return filt;
    }
    
    /**
     * @brief 滤波
     */
    double filter(double input) override {
        double output = input;
        for (auto& bq : biquads_) {
            output = bq.process(output);
        }
        return output * gain_;
    }
    
    /**
     * @brief 重置状态
     */
    void reset() override {
        for (auto& bq : biquads_) {
            bq.reset();
        }
    }
    
private:
    // Biquad 二阶节
    struct Biquad {
        double b0 = 1, b1 = 0, b2 = 0;
        double a1 = 0, a2 = 0;
        double z1 = 0, z2 = 0;  // 状态
        
        double process(double input) {
            // Direct Form II Transposed
            double output = b0 * input + z1;
            z1 = b1 * input - a1 * output + z2;
            z2 = b2 * input - a2 * output;
            return output;
        }
        
        void reset() { z1 = z2 = 0; }
    };
    
    std::vector<Biquad> biquads_;
    double gain_ = 1.0;
    
    void design(int order, double fc, double fs, FilterType type) {
        double wc = 2.0 * M_PI * fc / fs;
        double wc_warped = std::tan(wc / 2.0);
        
        biquads_.clear();
        gain_ = 1.0;
        
        // Butterworth 极点 (左半平面单位圆上均匀分布)
        int num_biquads = (order + 1) / 2;
        
        for (int k = 0; k < num_biquads; ++k) {
            Biquad bq;
            
            if (order % 2 == 1 && k == num_biquads - 1) {
                // 一阶节 (奇数阶的最后一个)
                double alpha = wc_warped;
                if (type == FilterType::Lowpass) {
                    double norm = 1.0 / (1.0 + alpha);
                    bq.b0 = alpha * norm;
                    bq.b1 = bq.b0;
                    bq.b2 = 0;
                    bq.a1 = (alpha - 1.0) * norm;
                    bq.a2 = 0;
                } else {  // Highpass
                    double norm = 1.0 / (1.0 + alpha);
                    bq.b0 = norm;
                    bq.b1 = -norm;
                    bq.b2 = 0;
                    bq.a1 = (alpha - 1.0) * norm;
                    bq.a2 = 0;
                }
            } else {
                // 二阶节
                double theta = M_PI * (2.0 * k + 1.0) / (2.0 * order);
                double sin_theta = std::sin(theta);
                double cos_theta = std::cos(theta);
                
                // s 域极点: s_k = wc * exp(j*(pi/2 + theta))
                // 使用 bilinear transform
                double wc2 = wc_warped * wc_warped;
                double alpha = 2.0 * wc_warped * sin_theta;
                
                if (type == FilterType::Lowpass) {
                    double norm = 1.0 / (1.0 + alpha + wc2);
                    bq.b0 = wc2 * norm;
                    bq.b1 = 2.0 * bq.b0;
                    bq.b2 = bq.b0;
                    bq.a1 = 2.0 * (wc2 - 1.0) * norm;
                    bq.a2 = (1.0 - alpha + wc2) * norm;
                } else {  // Highpass
                    double norm = 1.0 / (1.0 + alpha + wc2);
                    bq.b0 = norm;
                    bq.b1 = -2.0 * norm;
                    bq.b2 = norm;
                    bq.a1 = 2.0 * (wc2 - 1.0) * norm;
                    bq.a2 = (1.0 - alpha + wc2) * norm;
                }
            }
            
            biquads_.push_back(bq);
        }
    }
    
    void designBandpass(int order, double fl, double fh, double fs) {
        // 带通 = 低通 * 高通 (级联)
        double fc = std::sqrt(fl * fh);  // 中心频率
        double bw = fh - fl;
        
        // 简化设计：使用中心频率和Q因子
        design(order, fh, fs, FilterType::Lowpass);
        
        ButterworthFilter hp;
        hp.design(order, fl, fs, FilterType::Highpass);
        
        // 合并节
        for (const auto& bq : hp.biquads_) {
            biquads_.push_back(bq);
        }
        gain_ *= hp.gain_;
    }
};

/**
 * @brief 简单一阶低通滤波器 (RC)
 */
class FirstOrderLowpass : public IFilter {
public:
    /**
     * @brief 构造
     * @param cutoff_hz 截止频率
     * @param sample_hz 采样频率
     */
    FirstOrderLowpass(double cutoff_hz, double sample_hz) {
        double rc = 1.0 / (2.0 * M_PI * cutoff_hz);
        double dt = 1.0 / sample_hz;
        alpha_ = dt / (rc + dt);
    }
    
    /**
     * @brief 从时间常数构造
     */
    static FirstOrderLowpass fromTimeConstant(double tau, double sample_hz) {
        FirstOrderLowpass f(0, 1);  // 临时构造
        double dt = 1.0 / sample_hz;
        f.alpha_ = dt / (tau + dt);
        return f;
    }
    
    double filter(double input) override {
        y_ = alpha_ * input + (1.0 - alpha_) * y_;
        return y_;
    }
    
    void reset() override { y_ = 0; }
    
private:
    double alpha_ = 0;
    double y_ = 0;
};

} // namespace gnc::libraries::filters
