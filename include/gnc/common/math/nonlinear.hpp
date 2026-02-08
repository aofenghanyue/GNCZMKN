/**
 * @file nonlinear.hpp
 * @brief 非线性函数 - 有状态非线性模型
 * 
 * 包含:
 * - 速率限制器 (RateLimiter)
 * - 齿隙模型 (Backlash)
 * - 迟滞模型 (Hysteresis)
 * 
 * @note 与 special_functions.hpp 中的无状态函数 (sat, deadzone) 配合使用
 */
#pragma once

#include <cmath>
#include <algorithm>

namespace gnc::math {

/**
 * @brief 速率限制器
 * 
 * 限制信号的变化速率，常用于执行器模型
 */
class RateLimiter {
public:
    /**
     * @brief 构造速率限制器
     * @param rising_rate 上升速率限制 (正值)
     * @param falling_rate 下降速率限制 (正值，内部转为负值)
     * @param initial_value 初始输出值
     */
    RateLimiter(double rising_rate, double falling_rate, double initial_value = 0.0)
        : rising_rate_(rising_rate), falling_rate_(-std::abs(falling_rate)),
          prev_output_(initial_value) {}
    
    /**
     * @brief 对称速率限制器
     */
    explicit RateLimiter(double rate, double initial_value = 0.0)
        : RateLimiter(rate, rate, initial_value) {}
    
    /**
     * @brief 更新速率限制器
     * @param input 输入信号
     * @param dt 时间步长
     * @return 速率限制后的输出
     */
    double update(double input, double dt) {
        double delta = input - prev_output_;
        double max_rise = rising_rate_ * dt;
        double max_fall = falling_rate_ * dt;  // 负值
        
        // 限制变化量
        if (delta > max_rise) {
            delta = max_rise;
        } else if (delta < max_fall) {
            delta = max_fall;
        }
        
        prev_output_ += delta;
        return prev_output_;
    }
    
    /**
     * @brief 重置
     */
    void reset(double initial_value = 0.0) {
        prev_output_ = initial_value;
    }
    
    /**
     * @brief 获取当前输出
     */
    double output() const { return prev_output_; }
    
    /**
     * @brief 设置速率限制
     */
    void setRates(double rising, double falling) {
        rising_rate_ = rising;
        falling_rate_ = -std::abs(falling);
    }
    
private:
    double rising_rate_;    ///< 上升速率 (正值)
    double falling_rate_;   ///< 下降速率 (负值)
    double prev_output_;    ///< 上一输出
};

/**
 * @brief 齿隙/间隙模型 (Backlash)
 * 
 * 模拟机械齿轮间隙、传动带松弛等现象
 */
class Backlash {
public:
    /**
     * @brief 构造齿隙模型
     * @param deadband 死区宽度 (总宽度)
     * @param initial_output 初始输出
     */
    explicit Backlash(double deadband, double initial_output = 0.0)
        : half_deadband_(deadband / 2.0), prev_output_(initial_output) {}
    
    /**
     * @brief 更新齿隙模型
     * @param input 输入信号
     * @return 输出信号
     */
    double update(double input) {
        double upper = prev_output_ + half_deadband_;
        double lower = prev_output_ - half_deadband_;
        
        if (input > upper) {
            prev_output_ = input - half_deadband_;
        } else if (input < lower) {
            prev_output_ = input + half_deadband_;
        }
        // 在死区内，输出不变
        
        return prev_output_;
    }
    
    /**
     * @brief 重置
     */
    void reset(double initial_output = 0.0) {
        prev_output_ = initial_output;
    }
    
    /**
     * @brief 获取当前输出
     */
    double output() const { return prev_output_; }
    
    /**
     * @brief 设置死区宽度
     */
    void setDeadband(double deadband) {
        half_deadband_ = deadband / 2.0;
    }
    
private:
    double half_deadband_;  ///< 半死区宽度
    double prev_output_;    ///< 当前输出
};

/**
 * @brief 继电器迟滞模型 (Relay with Hysteresis)
 * 
 * 带迟滞的开关行为，常用于控制系统建模
 */
class RelayHysteresis {
public:
    /**
     * @brief 构造继电器迟滞模型
     * @param on_threshold 开启阈值 (输入从低到高超过此值时开启)
     * @param off_threshold 关闭阈值 (输入从高到低低于此值时关闭)
     * @param on_value 开启状态输出
     * @param off_value 关闭状态输出
     */
    RelayHysteresis(double on_threshold, double off_threshold,
                    double on_value = 1.0, double off_value = 0.0)
        : on_threshold_(on_threshold), off_threshold_(off_threshold),
          on_value_(on_value), off_value_(off_value), is_on_(false) {}
    
    /**
     * @brief 更新继电器状态
     * @param input 输入信号
     * @return 输出信号
     */
    double update(double input) {
        if (is_on_) {
            if (input < off_threshold_) {
                is_on_ = false;
            }
        } else {
            if (input > on_threshold_) {
                is_on_ = true;
            }
        }
        
        return is_on_ ? on_value_ : off_value_;
    }
    
    /**
     * @brief 获取当前状态
     */
    bool isOn() const { return is_on_; }
    
    /**
     * @brief 重置
     */
    void reset(bool initial_state = false) {
        is_on_ = initial_state;
    }
    
private:
    double on_threshold_;
    double off_threshold_;
    double on_value_;
    double off_value_;
    bool is_on_;
};

/**
 * @brief 量化器 (Quantizer)
 * 
 * 将连续信号量化为离散级别
 */
class Quantizer {
public:
    /**
     * @brief 构造量化器
     * @param step 量化步长
     */
    explicit Quantizer(double step) : step_(step) {}
    
    /**
     * @brief 量化信号
     * @param input 输入信号
     * @return 量化后的信号
     */
    double quantize(double input) const {
        return std::floor(input / step_ + 0.5) * step_;
    }
    
    double operator()(double input) const { return quantize(input); }
    
    /**
     * @brief 设置量化步长
     */
    void setStep(double step) { step_ = step; }
    
private:
    double step_;
};

/**
 * @brief 摩擦模型 (Coulomb + Viscous)
 * 
 * F = F_c * sign(v) + b * v
 */
class FrictionModel {
public:
    /**
     * @brief 构造摩擦模型
     * @param coulomb 库仑摩擦力 (正值)
     * @param viscous 粘性摩擦系数
     * @param stiction 静摩擦力 (可选，大于库仑摩擦)
     */
    FrictionModel(double coulomb, double viscous, double stiction = 0.0)
        : coulomb_(coulomb), viscous_(viscous), 
          stiction_(stiction > 0 ? stiction : coulomb) {}
    
    /**
     * @brief 计算摩擦力
     * @param velocity 速度
     * @param applied_force 施加力 (用于判断静摩擦)
     * @return 摩擦力 (与运动方向相反)
     */
    double compute(double velocity, double applied_force = 0.0) const {
        constexpr double v_thresh = 1e-6;
        
        if (std::abs(velocity) < v_thresh) {
            // 静止或接近静止，静摩擦
            if (std::abs(applied_force) <= stiction_) {
                return -applied_force;  // 完全抵消施加力
            } else {
                // 突破静摩擦
                double sign_f = (applied_force > 0) ? 1.0 : -1.0;
                return -sign_f * stiction_;
            }
        } else {
            // 运动状态
            double sign_v = (velocity > 0) ? 1.0 : -1.0;
            return -sign_v * coulomb_ - viscous_ * velocity;
        }
    }
    
private:
    double coulomb_;   ///< 库仑摩擦
    double viscous_;   ///< 粘性系数
    double stiction_;  ///< 静摩擦
};

} // namespace gnc::math
