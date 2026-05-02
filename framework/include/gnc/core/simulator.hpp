/**
 * @file simulator.hpp
 * @brief 仿真器
 */
#pragma once

#include "gnc/infrastructure/auto_data_logger.hpp"
#include "component_registry.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/infrastructure/dependency_validator.hpp"
#include "integrators/rk4_integrator.hpp"
#include "scoped_registry.hpp"
#include "gnc/infrastructure/simulation_summary.hpp"
#include "gnc/interfaces/i_continuous_system.hpp"
#include "gnc/interfaces/i_integrator.hpp"
#include "gnc/interfaces/i_termination_evaluator.hpp"
#include "gnc/common/logger.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
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

    Simulator() = default;
    ~Simulator() = default;
    
    // 禁止拷贝
    Simulator(const Simulator&) = delete;
    Simulator& operator=(const Simulator&) = delete;
    
    /// 获取组件注册表
    ComponentRegistry& getRegistry() { return registry_; }
    const ComponentRegistry& getRegistry() const { return registry_; }

    void resetAssemblyState() {
        registry_.clear();
        clearExecutionPlan();
        termination_reason_ = "completed";
        current_time_ = 0.0;
        is_initialized_ = false;
        phase_manager_ = ExecutionPhaseManager{};
    }
    
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

    void clearExecutionPlan() {
        for (auto& bucket : stage_buckets_) {
            bucket.clear();
        }
    }

    void addComponentToStage(ComponentBase* component, ExecutionStage stage) {
        if (!component) {
            return;
        }

        const auto index = stageBucketIndex(stage);
        if (index < 0) {
            return;
        }

        auto& bucket = stage_buckets_[static_cast<size_t>(index)];
        if (std::find(bucket.begin(), bucket.end(), component) == bucket.end()) {
            bucket.push_back(component);
        }
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

        initializeComponentsInOrder();

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

        if (config_.dt <= 0.0 || config_.duration < 0.0) {
            throw std::runtime_error(
                "Simulator requires dt > 0 and duration >= 0 before run().");
        }

        const int total_steps =
            static_cast<int>(std::llround(config_.duration / config_.dt));
        int executed_steps = 0;
        termination_reason_ = "completed";
        const auto wall_start = std::chrono::steady_clock::now();

        for (int step = 0; step <= total_steps; ++step) {
            current_time_ = step * config_.dt;
            publishCurrentState(step, current_time_);

            for (auto& callback : before_step_callbacks_) {
                callback(step, current_time_, config_.dt);
            }

            executeDiscreteStages(step);

            if (step > 0 || auto_logger_.shouldRecordInitialState()) {
                auto_logger_.recordStep(current_time_);
            }

            if (checkTerminationConditions(step, current_time_)) {
                executed_steps = step;
                goto simulation_end;
            }

            if (step == total_steps) {
                executed_steps = step;
                break;
            }

            integrateContinuousSystems(step);
            executed_steps = step + 1;
            current_time_ = executed_steps * config_.dt;
            setComponentTimes(current_time_, executed_steps);

            for (auto& callback : after_step_callbacks_) {
                callback(step, current_time_, config_.dt);
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
        executeDiscreteStages(step_index);
        integrateContinuousSystems(step_index);
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
    static constexpr size_t kStageCount = 9;

    static std::array<ExecutionStage, kStageCount> orderedStages() {
        return {ExecutionStage::Environment,
                ExecutionStage::Perturbation,
                ExecutionStage::VehicleInput,
                ExecutionStage::VehicleProcess,
                ExecutionStage::VehicleOutput,
                ExecutionStage::Interaction,
                ExecutionStage::Form,
                ExecutionStage::Termination,
                ExecutionStage::Summary};
    }

    static int stageBucketIndex(ExecutionStage stage) {
        switch (stage) {
        case ExecutionStage::Environment:
            return 0;
        case ExecutionStage::Perturbation:
            return 1;
        case ExecutionStage::VehicleInput:
            return 2;
        case ExecutionStage::VehicleProcess:
            return 3;
        case ExecutionStage::VehicleOutput:
            return 4;
        case ExecutionStage::Interaction:
            return 5;
        case ExecutionStage::Form:
            return 6;
        case ExecutionStage::Termination:
            return 7;
        case ExecutionStage::Summary:
            return 8;
        default:
            return -1;
        }
    }

    void initializeComponentsInOrder() {
        const auto perturbation_index = stageBucketIndex(ExecutionStage::Perturbation);
        std::vector<ComponentBase*> initialized;
        if (perturbation_index >= 0) {
            for (auto* component : orderedComponentsInBucket(
                     static_cast<size_t>(perturbation_index))) {
                LOG_INFO("Initializing component: {}", component->getName());
                component->initialize();
                initialized.push_back(component);
            }
        }

        for (auto* component : registry_.getAllComponents()) {
            if (std::find(initialized.begin(), initialized.end(), component) !=
                initialized.end()) {
                continue;
            }
            LOG_INFO("Initializing component: {}", component->getName());
            component->initialize();
        }
    }

    void executeStage(ExecutionStage stage, int step_index) {
        const auto index = stageBucketIndex(stage);
        if (index < 0) {
            return;
        }

        for (auto* component : orderedComponentsInBucket(static_cast<size_t>(index))) {
            executeComponent(component, step_index);
        }
    }

    void executeComponent(ComponentBase* component, int step_index) {
        component->setSimTimeInternal_(current_time_, step_index);
        component->clearDebugSnapshotInternal_();
        if (!component->shouldExecute(step_index)) {
            return;
        }

        auto* continuous_system = dynamic_cast<interfaces::IContinuousSystem*>(component);
        if (continuous_system) {
            return;
        }

        component->update(config_.dt);
    }

    void executeDiscreteStages(int step_index) {
        for (const auto stage : orderedStages()) {
            executeStage(stage, step_index);
        }
    }

    void integrateContinuousSystems(int step_index) {
        if (!integrator_) {
            return;
        }

        struct PendingState {
            interfaces::IContinuousSystem* system = nullptr;
            Eigen::VectorXd state;
        };

        std::vector<PendingState> pending_states;

        for (const auto stage : orderedStages()) {
            const auto index = stageBucketIndex(stage);
            if (index < 0) {
                continue;
            }

            for (auto* component : orderedComponentsInBucket(static_cast<size_t>(index))) {
                if (!component) {
                    continue;
                }
                component->setSimTimeInternal_(current_time_, step_index);
                if (!component->shouldExecute(step_index)) {
                    continue;
                }

                auto* continuous_system =
                    dynamic_cast<interfaces::IContinuousSystem*>(component);
                if (!continuous_system) {
                    continue;
                }

                Eigen::VectorXd next_state = continuous_system->getState();
                integrator_->step(
                    [continuous_system](double t,
                                        const Eigen::VectorXd& state,
                                        Eigen::VectorXd& dxdt) {
                        continuous_system->computeDerivatives(t, state, dxdt);
                    },
                    current_time_,
                    next_state,
                    config_.dt);
                pending_states.push_back({continuous_system, std::move(next_state)});
            }
        }

        for (auto& pending : pending_states) {
            pending.system->setState(pending.state);
        }
    }

    void setComponentTimes(double time, int step_index) {
        for (auto* component : registry_.getAllComponents()) {
            if (component) {
                component->setSimTimeInternal_(time, step_index);
            }
        }
    }

    void publishCurrentState(int step_index, double time) {
        setComponentTimes(time, step_index);

        auto components = registry_.getAllComponents();
        std::stable_sort(components.begin(),
                         components.end(),
                         [this](const ComponentBase* lhs,
                                const ComponentBase* rhs) {
                             const int lhs_phase = publishPhaseRank(publishPhaseFor(lhs));
                             const int rhs_phase = publishPhaseRank(publishPhaseFor(rhs));
                             if (lhs_phase != rhs_phase) {
                                 return lhs_phase < rhs_phase;
                             }
                             return lhs->getPriority() < rhs->getPriority();
                         });

        for (auto* component : components) {
            if (component) {
                component->publish(time);
            }
        }
    }

    std::vector<ComponentBase*> orderedComponentsInBucket(size_t index) const {
        auto components = stage_buckets_[index];
        std::stable_sort(components.begin(),
                         components.end(),
                         [](const ComponentBase* lhs, const ComponentBase* rhs) {
                             return lhs->getPriority() < rhs->getPriority();
                         });
        return components;
    }

    PublishPhase publishPhaseFor(const ComponentBase* component) const {
        if (!component) {
            return PublishPhase::Ordinary;
        }
        if (dynamic_cast<const interfaces::IContinuousSystem*>(component)) {
            return PublishPhase::StateOwner;
        }
        return component->getPublishPhase();
    }

    static int publishPhaseRank(PublishPhase phase) {
        return static_cast<int>(phase);
    }

    bool checkTerminationConditions(int step_index, double time) {
        const auto index = stageBucketIndex(ExecutionStage::Termination);
        if (index < 0) {
            return false;
        }

        for (auto* component : orderedComponentsInBucket(static_cast<size_t>(index))) {
            auto* evaluator =
                dynamic_cast<interfaces::ITerminationEvaluator*>(component);
            if (evaluator && evaluator->shouldTerminate()) {
                termination_reason_ = evaluator->reason();
                if (termination_reason_.empty() && component) {
                    termination_reason_ = component->getName();
                }
                LOG_INFO("Simulation terminated early by condition '{}' at t={} s, step={}",
                         termination_reason_, time, step_index);
                return true;
            }
        }
        return false;
    }

    /// 根据频率计算各组件的步长间隔
    void computeStepIntervals() {
        if (config_.dt <= 0.0) {
            return;
        }
        double sim_freq = 1.0 / config_.dt;
        
        for (auto* component : registry_.getAllComponents()) {
            double comp_freq = component->getExecutionFrequency();
            
            int interval = 1;
            if (comp_freq > 0.0) {
                if (comp_freq > sim_freq) {
                    throw std::runtime_error(
                        "Component '" + component->getName() +
                        "' execution frequency exceeds simulation frequency.");
                }

                const double raw_interval = sim_freq / comp_freq;
                const double rounded_interval = std::round(raw_interval);
                const double tolerance =
                    1.0e-9 * std::max(1.0, std::abs(raw_interval));
                if (std::abs(raw_interval - rounded_interval) > tolerance ||
                    rounded_interval < 1.0) {
                    throw std::runtime_error(
                        "Component '" + component->getName() +
                        "' execution frequency must divide the simulation frequency into "
                        "an integer step interval.");
                }
                interval = static_cast<int>(rounded_interval);
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
    std::string termination_reason_ = "completed";
    double current_time_ = 0.0;
    bool is_initialized_ = false;
    std::array<std::vector<ComponentBase*>, kStageCount> stage_buckets_{};
};

} // namespace gnc::core
