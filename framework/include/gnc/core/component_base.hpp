/**
 * @file component_base.hpp
 * @brief 组件基类
 */
#pragma once

#include "gnc/core/config_manager.hpp"
#include "gnc/core/service_context.hpp"

#include <map>
#include <string>

namespace gnc::core {

class ScopedRegistry;

enum class PublishPhase {
    StateOwner = 0,
    View = 1,
    Ordinary = 2
};

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

    /// 发布当前周期开始的状态视图。
    ///
    /// Simulator 在每个固定步周期开始调用该钩子，并保证组件的 sim time
    /// 已设置为该周期发布时刻 t_k。连续组件和 observer 应在这里刷新
    /// truth/view/派生量；该钩子不应推进连续状态。
    virtual void publish(double time) { (void)time; }
    
    /// 更新（每个仿真周期最多调用一次，由调度器根据频率决定是否调用）。
    /// update 读取已发布的 t_k 状态，产生用于 [t_k, t_{k+1}] 的本周期离散输出。
    /// 连续状态推进由积分器通过 IContinuousSystem 完成，不应在 update 中积分。
    virtual void update(double dt) = 0;
    
    /// 终结（仿真结束时调用）
    virtual void finalize() {}
    
    // --- 配置 ---
    
    /// 从JSON配置节点加载组件参数（由MissionAssembler在构建期调用，先于注册完成）
    virtual void configure(const ConfigNode& config) { (void)config; }

    /// 带配置路径的加载入口。新组件可覆盖该函数以生成更精确的诊断；
    /// 旧组件继续覆盖 configure(config) 即可。
    virtual void configure(const ConfigNode& config, const std::string& config_path) {
        (void)config_path;
        configure(config);
    }
    
    // --- 依赖注入 ---
    
    /// 注入组件依赖（构建期会先做一次预检；若尚未成功，Simulator会在initialize前再次尝试）
    // Dependency injection should stay side-effect free. The framework may preflight
    // required bindings during build and skip reinjection later if it already succeeded.
    virtual void injectDependencies(ScopedRegistry& registry) { (void)registry; }

    /// 注入服务依赖（由MissionAssembler在构建期调用，顺序为 global -> environment -> vehicle）
    virtual void injectServices(ServiceContext& services) { (void)services; }
    
    // --- 元数据 ---
    
    const std::string& getName() const { return name_; }
    const std::string& getTypeName() const { return type_name_; }
    const std::string& getComponentCategory() const { return component_category_; }
    const std::string& getRegistrationOrigin() const { return registration_origin_; }
    virtual PublishPhase getPublishPhase() const { return PublishPhase::Ordinary; }
    int getPriority() const { return priority_; }

    /// 获取当前步由组件主动写入的调试快照
    const std::map<std::string, double>& getDebugSnapshot() const {
        return debug_snapshot_;
    }
    
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

    double getSimTime() const { return sim_time_; }
    int getSimStep() const { return sim_step_; }

    void setSimTimeInternal_(double time, int step) {
        sim_time_ = time;
        sim_step_ = step;
    }

    void setNameInternal_(const std::string& name) {
        name_ = name;
    }

    void setTypeNameInternal_(const std::string& type_name) {
        type_name_ = type_name;
    }

    void setComponentCategoryInternal_(const std::string& category) {
        component_category_ = category;
    }

    void setRegistrationOriginInternal_(const std::string& origin) {
        registration_origin_ = origin;
    }

    void setPriorityInternal_(int priority) {
        priority_ = priority;
    }

    void clearDebugSnapshotInternal_() {
        debug_snapshot_.clear();
    }

    bool dependenciesInjectedInternal_() const {
        return dependencies_injected_;
    }

    void markDependenciesInjectedInternal_() {
        dependencies_injected_ = true;
    }

    void resetDependenciesInjectedInternal_() {
        dependencies_injected_ = false;
    }

protected:
    /// 记录当前步的调试标量，不要求它成为稳定可观测字段
    void snapDebug(const std::string& name, double value) {
        debug_snapshot_[name] = value;
    }
    
private:
    std::string name_;
    std::string type_name_;
    std::string component_category_ = "project";
    std::string registration_origin_;
    int priority_ = 0;
    double freq_hz_ = 0.0;      // 0表示每步执行
    int step_interval_ = 1;      // 执行步长间隔
    double sim_time_ = 0.0;
    int sim_step_ = 0;
    std::map<std::string, double> debug_snapshot_;
    bool dependencies_injected_ = false;
};

} // namespace gnc::core
