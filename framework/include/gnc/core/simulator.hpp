/**
 * @file simulator.hpp
 * @brief 仿真器
 */
#pragma once

#include "gnc/infrastructure/auto_data_logger.hpp"
#include "component_registry.hpp"
#include "gnc/infrastructure/dependency_validator.hpp"
#include "integrators/rk4_integrator.hpp"
#include "scoped_registry.hpp"
#include "gnc/infrastructure/simulation_summary.hpp"
#include "gnc/interfaces/i_continuous_system.hpp"
#include "gnc/interfaces/i_integrator.hpp"
#include "gnc/common/logger.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

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
    using StepCallback = std::function<void(int step, double time, double dt)>;
    using TerminationCondition = std::function<bool(int step, double time)>;

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

    void setIntegrator(std::unique_ptr<interfaces::IIntegrator> integrator) {
        integrator_ = std::move(integrator);
    }

    const interfaces::IIntegrator* getIntegrator() const {
        return integrator_.get();
    }

    bool initializeAutoDataLogger(const ConfigNode& config) {
        return auto_logger_.initialize(config, registry_);
    }

    AutoDataLogger& getAutoDataLogger() { return auto_logger_; }
    const AutoDataLogger& getAutoDataLogger() const { return auto_logger_; }

    void onBeforeStep(StepCallback callback) {
        before_step_callbacks_.push_back(std::move(callback));
    }

    void onAfterStep(StepCallback callback) {
        after_step_callbacks_.push_back(std::move(callback));
    }

    void addTerminationCondition(const std::string& name,
                                 TerminationCondition condition) {
        termination_conditions_.push_back({name, std::move(condition)});
    }

    const std::string& getTerminationReason() const {
        return termination_reason_;
    }
    
    /// 初始化仿真
    void initialize() {
        phase_manager_.transitionTo(ExecutionPhaseManager::Phase::Initializing);
        LOG_INFO("Initializing simulator...");
        current_time_ = 0.0;
        
        computeStepIntervals();

        for (auto* component : registry_.getAllComponents()) {
            if (component->dependenciesInjectedInternal_()) {
                continue;
            }
            std::string scope = extractScope(component->getName());
            ScopedRegistry scoped(scope, registry_, component->getName());
            component->injectDependencies(scoped);
            component->markDependenciesInjectedInternal_();
        }

        for (auto* component : registry_.getAllComponents()) {
            LOG_INFO("Initializing component: {}", component->getName());
            component->initialize();
        }

        if (!integrator_) {
            integrator_ = std::make_unique<RK4Integrator>();
        }
        
        is_initialized_ = true;
        LOG_INFO("Simulator initialized with {} components using '{}' integrator",
                 registry_.size(), integrator_->name());
    }
    
    /// 运行仿真
    void run() {
        if (!is_initialized_) {
            initialize();
        }

        LOG_INFO("Starting simulation: dt={}, duration={}", config_.dt, config_.duration);
        phase_manager_.transitionTo(ExecutionPhaseManager::Phase::Running);

        const int total_steps = static_cast<int>(config_.duration / config_.dt);
        int executed_steps = 0;
        termination_reason_ = "completed";
        const auto wall_start = std::chrono::steady_clock::now();

        for (int step = 0; step < total_steps; ++step) {
            for (auto& callback : before_step_callbacks_) {
                callback(step, current_time_, config_.dt);
            }

            this->step(step);
            auto_logger_.recordStep(current_time_);

            for (auto& callback : after_step_callbacks_) {
                callback(step, current_time_, config_.dt);
            }

            current_time_ += config_.dt;
            executed_steps = step + 1;

            for (const auto& condition : termination_conditions_) {
                if (condition.condition(step, current_time_)) {
                    termination_reason_ = condition.name;
                    LOG_INFO("Simulation terminated early by condition '{}' at t={} s, step={}",
                             condition.name, current_time_, step);
                    goto simulation_end;
                }
            }
        }

simulation_end:
        {
            const auto wall_end = std::chrono::steady_clock::now();
            const double wall_seconds = std::chrono::duration<double>(wall_end - wall_start).count();
            auto_logger_.stop();

            if (!auto_logger_.getOutputDir().empty()) {
                SimulationSummary::SimInfo info;
                info.dt = config_.dt;
                info.duration = config_.duration;
                info.final_time = current_time_;
                info.total_steps = executed_steps;
                info.termination_reason = termination_reason_;
                info.wall_clock_seconds = wall_seconds;
                SimulationSummary::write(auto_logger_.getOutputDir(), info, registry_, auto_logger_);
            }
        }

        phase_manager_.transitionTo(ExecutionPhaseManager::Phase::Finalizing);
        finalize();
        phase_manager_.transitionTo(ExecutionPhaseManager::Phase::Completed);
        LOG_INFO("Simulation completed. Final time: {}, reason: {}", current_time_, termination_reason_);
    }
    
    /// 单步执行
    void step(int step_index) {
        for (auto* component : registry_.getAllComponents()) {
            component->setSimTimeInternal_(current_time_, step_index);
            component->clearDebugSnapshotInternal_();
            if (!component->shouldExecute(step_index)) {
                continue;
            }

            auto* continuous_system = dynamic_cast<interfaces::IContinuousSystem*>(component);
            if (continuous_system && integrator_) {
                Eigen::VectorXd x = continuous_system->getState();
                integrator_->step(
                    [continuous_system](double t,
                                        const Eigen::VectorXd& state,
                                        Eigen::VectorXd& dxdt) {
                        continuous_system->computeDerivatives(t, state, dxdt);
                    },
                    current_time_,
                    x,
                    config_.dt
                );
                continuous_system->setState(x);
                component->update(config_.dt);
            } else {
                component->update(config_.dt);
            }
        }
    }
    
    /// 终结仿真
    void finalize() {
        LOG_INFO("Finalizing simulator...");
        for (auto* component : registry_.getAllComponents()) {
            component->finalize();
        }
    }
    
    /// 获取当前仿真时间
    double getCurrentTime() const { return current_time_; }
    
private:
    struct NamedCondition {
        std::string name;
        TerminationCondition condition;
    };

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
    AutoDataLogger auto_logger_;
    std::unique_ptr<interfaces::IIntegrator> integrator_;
    ExecutionPhaseManager phase_manager_;
    std::vector<StepCallback> before_step_callbacks_;
    std::vector<StepCallback> after_step_callbacks_;
    std::vector<NamedCondition> termination_conditions_;
    std::string termination_reason_ = "completed";
    double current_time_ = 0.0;
    bool is_initialized_ = false;
};

} // namespace gnc::core
