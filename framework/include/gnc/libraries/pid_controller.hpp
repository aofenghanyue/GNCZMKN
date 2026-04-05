/**
 * @file pid_controller.hpp
 * @brief 增强型 PID 控制器库
 * 
 * 特性:
 * - 抗积分饱和 (Anti-Windup)
 * - 输出限幅
 * - 微分滤波
 * - 前馈
 */
#pragma once

#include <cmath>
#include <algorithm>

namespace gnc::libraries {

/**
 * @brief 抗积分饱和策略
 */
enum class AntiWindup {
    None,            ///< 无抗积分饱和
    Clamp,           ///< 简单钳位 (积分项限幅)
    BackCalculation  ///< 回算法 (推荐)
};

/**
 * @brief PID 控制器配置
 */
struct PidConfig {
    double kp = 1.0;              ///< 比例增益
    double ki = 0.0;              ///< 积分增益
    double kd = 0.0;              ///< 微分增益
    
    double output_min = -1e30;    ///< 输出下限
    double output_max = 1e30;     ///< 输出上限
    
    double integral_min = -1e30;  ///< 积分项下限
    double integral_max = 1e30;   ///< 积分项上限
    
    AntiWindup anti_windup = AntiWindup::Clamp;  ///< 抗积分饱和策略
    double kb = 0.0;              ///< 回算增益 (用于 BackCalculation)
    
    double deriv_filter_tau = 0.0;  ///< 微分滤波时间常数 (0 表示不滤波)
};

/**
 * @brief 增强型 PID 控制器
 * 
 * 输出 = Kp * e + Ki * ∫e dt + Kd * de/dt + feedforward
 */
class PidController {
public:
    /**
     * @brief 默认构造
     */
    PidController() = default;
    
    /**
     * @brief 简单构造
     */
    PidController(double kp, double ki = 0.0, double kd = 0.0)
        : config_{kp, ki, kd} {}
    
    /**
     * @brief 带配置构造
     */
    explicit PidController(const PidConfig& config) : config_(config) {}
    
    /**
     * @brief 设置增益
     */
    void setGains(double kp, double ki, double kd) {
        config_.kp = kp;
        config_.ki = ki;
        config_.kd = kd;
    }
    
    /**
     * @brief 设置输出限幅
     */
    void setOutputLimits(double min_val, double max_val) {
        config_.output_min = min_val;
        config_.output_max = max_val;
    }
    
    /**
     * @brief 设置积分限幅
     */
    void setIntegralLimits(double min_val, double max_val) {
        config_.integral_min = min_val;
        config_.integral_max = max_val;
    }
    
    /**
     * @brief 设置抗积分饱和模式
     */
    void setAntiWindupMode(AntiWindup mode) {
        config_.anti_windup = mode;
    }
    
    /**
     * @brief 设置回算增益 (用于 BackCalculation 模式)
     */
    void setBackCalculationGain(double kb) {
        config_.kb = kb;
    }
    
    /**
     * @brief 设置微分滤波时间常数
     */
    void setDerivativeFilter(double tau) {
        config_.deriv_filter_tau = tau;
    }
    
    /**
     * @brief 设置完整配置
     */
    void setConfig(const PidConfig& config) {
        config_ = config;
    }
    
    /**
     * @brief 重置控制器状态
     */
    void reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
        prev_deriv_ = 0.0;
        first_update_ = true;
    }
    
    /**
     * @brief 计算 PID 输出
     * @param error 误差 (setpoint - measurement)
     * @param dt 时间步长
     * @return 控制输出
     */
    double compute(double error, double dt) {
        if (dt <= 0.0) return 0.0;
        
        // 比例项
        double p_term = config_.kp * error;
        
        // 积分项
        double i_term = 0.0;
        if (config_.ki != 0.0) {
            integral_ += error * dt;
            
            // 积分限幅 (Clamp 模式)
            if (config_.anti_windup == AntiWindup::Clamp) {
                integral_ = std::clamp(integral_, config_.integral_min, config_.integral_max);
            }
            
            i_term = config_.ki * integral_;
        }
        
        // 微分项
        double d_term = 0.0;
        if (config_.kd != 0.0 && !first_update_) {
            double deriv = (error - prev_error_) / dt;
            
            // 微分滤波 (一阶低通)
            if (config_.deriv_filter_tau > 0.0) {
                double alpha = dt / (config_.deriv_filter_tau + dt);
                deriv = prev_deriv_ + alpha * (deriv - prev_deriv_);
            }
            
            prev_deriv_ = deriv;
            d_term = config_.kd * deriv;
        }
        first_update_ = false;
        prev_error_ = error;
        
        // 计算未限幅输出
        double output_unrestricted = p_term + i_term + d_term;
        
        // 输出限幅
        double output = std::clamp(output_unrestricted, config_.output_min, config_.output_max);
        
        // 回算法抗积分饱和
        if (config_.anti_windup == AntiWindup::BackCalculation && config_.ki != 0.0) {
            double saturation_error = output - output_unrestricted;
            double kb = (config_.kb > 0.0) ? config_.kb : 1.0 / config_.ki;
            integral_ += kb * saturation_error * dt;
        }
        
        return output;
    }
    
    /**
     * @brief 计算 PID 输出 (带前馈)
     * @param error 误差
     * @param dt 时间步长
     * @param feedforward 前馈量
     */
    double compute(double error, double dt, double feedforward) {
        return compute(error, dt) + feedforward;
    }
    
    /**
     * @brief 获取内部状态 (用于调试)
     */
    double getIntegral() const { return integral_; }
    double getPrevError() const { return prev_error_; }
    const PidConfig& getConfig() const { return config_; }
    
private:
    PidConfig config_;
    double integral_ = 0.0;
    double prev_error_ = 0.0;
    double prev_deriv_ = 0.0;
    bool first_update_ = true;
};

/**
 * @brief 增量式 PID 控制器
 * 
 * 输出增量 Δu = Kp*(e - e_1) + Ki*e + Kd*(e - 2*e_1 + e_2)
 */
class IncrementalPidController {
public:
    IncrementalPidController(double kp = 1.0, double ki = 0.0, double kd = 0.0)
        : kp_(kp), ki_(ki), kd_(kd) {}
    
    void setGains(double kp, double ki, double kd) {
        kp_ = kp; ki_ = ki; kd_ = kd;
    }
    
    void reset() {
        e1_ = 0.0;
        e2_ = 0.0;
        output_ = 0.0;
        first_ = true;
    }
    
    /**
     * @brief 计算输出增量
     * @return 控制输出 (累积值)
     */
    double compute(double error, double /*dt*/) {
        double delta;
        if (first_) {
            delta = kp_ * error;  // 首次只有比例
            first_ = false;
        } else {
            delta = kp_ * (error - e1_) + ki_ * error + kd_ * (error - 2*e1_ + e2_);
        }
        
        e2_ = e1_;
        e1_ = error;
        output_ += delta;
        
        return output_;
    }
    
    double getOutput() const { return output_; }
    
private:
    double kp_, ki_, kd_;
    double e1_ = 0.0, e2_ = 0.0;
    double output_ = 0.0;
    bool first_ = true;
};

} // namespace gnc::libraries
