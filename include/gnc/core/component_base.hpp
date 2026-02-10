/**
 * @file component_base.hpp
 * @brief 组件基类
 */
#pragma once

#include <string>

namespace gnc::core {

class ScopedRegistry;
class ServiceContext;
class ConfigNode;

/**
 * @brief 组件基类
 * 
 * 所有仿真组件的基类，提供：
 * - 生命周期管理 (initialize, update, finalize)
 * - 执行频率控制
 * - 依赖注入接口（组件 + 服务）
 * - 配置透传接口
 */
class ComponentBase {
public:
    explicit ComponentBase(const std::string& name) : name_(name) {}
    virtual ~ComponentBase() = default;
    
    // 禁止拷贝和移动
    ComponentBase(const ComponentBase&) = delete;
    ComponentBase& operator=(const ComponentBase&) = delete;
    ComponentBase(ComponentBase&&) = delete;
    ComponentBase& operator=(ComponentBase&&) = delete;
    
    // --- 生命周期 ---
    
    /// 初始化（在所有组件注册后、首次更新前调用）
    virtual void initialize() {}
    
    /// 更新（每个仿真步调用，由调度器根据频率决定是否调用）
    virtual void update(double dt) = 0;
    
    /// 终结（仿真结束时调用）
    virtual void finalize() {}
    
    // --- 配置 ---
    
    /// 从JSON配置节点加载组件参数（由SimulationBuilder在注册后调用）
    virtual void configure(const ConfigNode& config) { (void)config; }
    
    // --- 依赖注入 ---
    
    /// 注入组件依赖（由Simulator在initialize前调用）
    virtual void injectDependencies(ScopedRegistry& registry) { (void)registry; }
    
    /// 注入服务依赖（由Simulator在injectDependencies后调用）
    virtual void injectServices(ServiceContext& services) { (void)services; }
    
    // --- 元数据 ---
    
    const std::string& getName() const { return name_; }
    
    // --- 执行频率控制 ---
    
    /// 设置执行频率 (Hz)，0表示每步都执行
    void setExecutionFrequency(double freq_hz) { freq_hz_ = freq_hz; }
    double getExecutionFrequency() const { return freq_hz_; }
    
    /// 内部使用：设置步长间隔
    void setStepInterval(int interval) { step_interval_ = interval; }
    int getStepInterval() const { return step_interval_; }
    
    /// 内部使用：检查本周期是否应执行
    bool shouldExecute(int current_step) const {
        if (step_interval_ <= 1) return true;
        return (current_step % step_interval_) == 0;
    }
    
private:
    std::string name_;
    double freq_hz_ = 0.0;      // 0表示每步执行
    int step_interval_ = 1;      // 执行步长间隔
};

} // namespace gnc::core
