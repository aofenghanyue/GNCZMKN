/**
 * @file simple_navigation.hpp
 * @brief 简单导航实现
 */
#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/interfaces/gnc/i_navigation.hpp"
#include "gnc/interfaces/sensors/i_imu_sensor.hpp"

namespace gnc::components {

/**
 * @brief 简单导航解算
 * 
 * 占位实现，展示依赖注入机制
 */
class SimpleNavigation : public core::ComponentBase,
                          public interfaces::INavigation {
public:
    SimpleNavigation() : ComponentBase("SimpleNavigation") {
        setExecutionFrequency(50.0);  // 50 Hz
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
    
    void injectDependencies(core::ComponentRegistry& registry) override {
        imu_ = registry.getFirst<interfaces::IImuSensor>();
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
        nav_state_.timestamp = imu_data.timestamp;
    }
    
private:
    interfaces::NavState nav_state_;
    interfaces::IImuSensor* imu_ = nullptr;
    bool is_valid_ = false;
};

// 自动注册到工厂
GNC_REGISTER_COMPONENT(SimpleNavigation, interfaces::INavigation)

} // namespace gnc::components
