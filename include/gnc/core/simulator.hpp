/**
 * @file simulator.hpp
 * @brief 仿真器
 */
#pragma once

#include "component_registry.hpp"
#include "scoped_registry.hpp"
#include "gnc/common/logger.hpp"
#include <memory>
#include <vector>
#include <cmath>

namespace gnc::core {

/**
 * @brief 仿真器配置
 */
struct SimulatorConfig {
    double dt = 0.01;           // 仿真步长 (s)
    double duration = 10.0;     // 仿真时长 (s)
};

/**
 * @brief 仿真器
 * 
 * 负责：
 * - 管理组件注册表
 * - 组件生命周期调度
 * - 仿真主循环
 */
class Simulator {
public:
    Simulator() = default;
    ~Simulator() = default;
    
    // 禁止拷贝
    Simulator(const Simulator&) = delete;
    Simulator& operator=(const Simulator&) = delete;
    
    /// 获取组件注册表
    ComponentRegistry& getRegistry() { return registry_; }
    const ComponentRegistry& getRegistry() const { return registry_; }
    
    /// 设置仿真配置
    void configure(const SimulatorConfig& config) {
        config_ = config;
        computeStepIntervals();
    }
    
    /// 初始化仿真
    void initialize() {
        log_info("Initializing simulator...");
        
        // 计算组件执行间隔
        computeStepIntervals();
        
        // 注入依赖，根据组件名前缀创建对应的 ScopedRegistry
        for (auto* component : registry_.getAllComponents()) {
            std::string scope = extractScope(component->getName());
            ScopedRegistry scoped(scope, registry_);
            component->injectDependencies(scoped);
        }
        
        // 初始化所有组件
        for (auto* component : registry_.getAllComponents()) {
            log_info("Initializing component: {}", component->getName());
            component->initialize();
        }
        
        is_initialized_ = true;
        log_info("Simulator initialized with {} components", registry_.size());
    }
    
    /// 运行仿真
    void run() {
        if (!is_initialized_) {
            initialize();
        }
        
        log_info("Starting simulation: dt={}, duration={}", config_.dt, config_.duration);
        
        int total_steps = static_cast<int>(config_.duration / config_.dt);
        
        for (int step = 0; step < total_steps; ++step) {
            this->step(step);
            current_time_ += config_.dt;
        }
        
        finalize();
        log_info("Simulation completed. Final time: {}", current_time_);
    }
    
    /// 单步执行
    void step(int step_index) {
        for (auto* component : registry_.getAllComponents()) {
            if (component->shouldExecute(step_index)) {
                component->update(config_.dt);
            }
        }
    }
    
    /// 终结仿真
    void finalize() {
        log_info("Finalizing simulator...");
        for (auto* component : registry_.getAllComponents()) {
            component->finalize();
        }
    }
    
    /// 获取当前仿真时间
    double getCurrentTime() const { return current_time_; }
    
private:
    /// 根据频率计算各组件的步长间隔
    void computeStepIntervals() {
        double sim_freq = 1.0 / config_.dt;
        
        for (auto* component : registry_.getAllComponents()) {
            double comp_freq = component->getExecutionFrequency();
            
            int interval = 1;
            if (comp_freq > 0 && comp_freq < sim_freq) {
                interval = static_cast<int>(std::round(sim_freq / comp_freq));
                interval = std::max(1, interval);
            }
            
            component->setStepInterval(interval);
        }
    }
    
    /// 从组件全名提取作用域前缀（如 "chaser.imu" → "chaser."）
    static std::string extractScope(const std::string& fullName) {
        auto pos = fullName.find('.');
        if (pos == std::string::npos) {
            return "";  // 无前缀，返回空作用域
        }
        return fullName.substr(0, pos + 1);  // 包含 '.'
    }
    
    ComponentRegistry registry_;
    SimulatorConfig config_;
    double current_time_ = 0.0;
    bool is_initialized_ = false;
};

} // namespace gnc::core
