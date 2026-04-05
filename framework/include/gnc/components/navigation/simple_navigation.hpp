/**
 * @file simple_navigation.hpp
 * @brief 简单导航实现
 */
#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/core/dependency_validator.hpp"
#include "gnc/core/observable_helpers.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/interfaces/gnc/i_navigation.hpp"
#include "gnc/interfaces/infrastructure/i_observable.hpp"
#include "gnc/interfaces/sensors/i_imu_sensor.hpp"

namespace gnc::components {

/**
 * @brief 简单导航解算
 * 
 * 占位实现，展示依赖注入机制
 */
class SimpleNavigation : public core::ComponentBase,
                         public interfaces::INavigation,
                         public core::IDependencyDeclarer,
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

    std::vector<core::DependencyDeclaration> getDependencies() const override {
        return {
            {std::type_index(typeid(interfaces::IImuSensor)), "an IMU sensor interface", true}
        };
    }
    
    // --- ComponentBase 生命周期 ---
    
    void injectDependencies(core::ScopedRegistry& registry) override {
        imu_ = registry.getByName<interfaces::IImuSensor>("imu");
    }
    
    void initialize() override {
        is_valid_ = (imu_ != nullptr);
    }
    
    void update(double dt) override {
        if (!imu_) return;
        
        const auto& imu_data = imu_->getImuData();
        
        // TODO: 实现真正的导航滤波
        // 简单积分示例
        nav_state_.velocity += imu_data.acceleration * dt;
        nav_state_.position += nav_state_.velocity * dt;
        nav_state_.angular_velocity = imu_data.angular_velocity;
        nav_state_.timestamp = getSimTime();
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
    interfaces::IImuSensor* imu_ = nullptr;
    bool is_valid_ = false;
};

// 自动注册到工厂
GNC_REGISTER_COMPONENT(SimpleNavigation, interfaces::INavigation)

} // namespace gnc::components
