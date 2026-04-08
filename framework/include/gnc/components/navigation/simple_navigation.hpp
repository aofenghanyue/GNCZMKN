/**
 * @file simple_navigation.hpp
 * @brief 简单导航实现
 */
#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/core/observable_helpers.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/interfaces/gnc/i_navigation.hpp"
#include "gnc/interfaces/infrastructure/i_observable.hpp"
#include "gnc/interfaces/state/i_angular_velocity_provider.hpp"
#include "gnc/interfaces/state/i_attitude_provider.hpp"
#include "gnc/interfaces/state/i_position_provider.hpp"
#include "gnc/interfaces/state/i_velocity_provider.hpp"

namespace gnc::components {

/**
 * @brief 简单导航解算
 * 
 * 占位实现，展示依赖注入机制
 */
class SimpleNavigation : public core::ComponentBase,
                         public interfaces::INavigation,
                         public interfaces::IObservable {
public:
    SimpleNavigation() : ComponentBase("SimpleNavigation") {
        setExecutionFrequency(50.0);  // 50 Hz
    }

    void configure(const core::ConfigNode& config) override {
        if (config.isNull()) {
            return;
        }
        const auto& frequency_hz = config["frequency_hz"];
        if (!frequency_hz.isNull()) {
            setExecutionFrequency(frequency_hz.asDouble(50.0));
            return;
        }
        const auto& frequency = config["frequency"];
        if (!frequency.isNull()) {
            setExecutionFrequency(frequency.asDouble(50.0));
        }
    }
    
    // --- INavigation 接口实现 ---
    
    const interfaces::NavState& getNavState() const override {
        return nav_state_;
    }
    
    double getTimestamp() const override {
        return nav_state_.timestamp;
    }
    
    bool isValid() const override {
        return is_valid_;
    }

    // --- ComponentBase 生命周期 ---
    
    void injectDependencies(core::ScopedRegistry& registry) override {
        registry.bindAll(
            core::bind(position_provider_, "dynamics"),
            core::bind(velocity_provider_, "dynamics"),
            core::bindIfPresent(attitude_provider_, "dynamics"),
            core::bindIfPresent(angular_velocity_provider_, "dynamics"));
    }
    
    void initialize() override {
        is_valid_ = (position_provider_ != nullptr && velocity_provider_ != nullptr);
    }
    
    void update(double dt) override {
        (void)dt;
        if (!position_provider_ || !velocity_provider_) {
            is_valid_ = false;
            return;
        }

        nav_state_.position = position_provider_->getPosition();
        nav_state_.velocity = velocity_provider_->getVelocity();
        nav_state_.attitude = attitude_provider_ ? attitude_provider_->getAttitude()
                                                 : Quaterniond::Identity();
        nav_state_.angular_velocity = angular_velocity_provider_
                                        ? angular_velocity_provider_->getAngularVelocity()
                                        : Vector3d::Zero();
        nav_state_.timestamp = getSimTime();
        is_valid_ = true;
    }

    std::vector<interfaces::ObservableField> getObservableFields() const override {
        core::ObservableFieldBuilder builder;
        builder.addVector3d("position", [this]() -> const Vector3d& { return nav_state_.position; });
        builder.addVector3d("velocity", [this]() -> const Vector3d& { return nav_state_.velocity; });
        builder.addQuaterniond("attitude", [this]() -> const Quaterniond& { return nav_state_.attitude; });
        builder.addVector3d("angular_velocity", [this]() -> const Vector3d& { return nav_state_.angular_velocity; });
        builder.addScalar("timestamp", [this]() { return nav_state_.timestamp; });
        return builder.build();
    }
    
private:
    interfaces::NavState nav_state_;
    interfaces::IPositionProvider* position_provider_ = nullptr;
    interfaces::IVelocityProvider* velocity_provider_ = nullptr;
    interfaces::IAttitudeProvider* attitude_provider_ = nullptr;
    interfaces::IAngularVelocityProvider* angular_velocity_provider_ = nullptr;
    bool is_valid_ = false;
};

// 自动注册到工厂
GNC_REGISTER_STARTER_COMPONENT(SimpleNavigation, interfaces::INavigation)

} // namespace gnc::components
