/**
 * @file ideal_imu.hpp
 * @brief 理想IMU传感器实现
 */
#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/core/dependency_validator.hpp"
#include "gnc/core/observable_helpers.hpp"
#include "gnc/interfaces/sensors/i_imu_sensor.hpp"
#include "gnc/interfaces/dynamics/i_dynamics.hpp"
#include "gnc/interfaces/infrastructure/i_observable.hpp"

namespace gnc::components {

/**
 * @brief 理想IMU传感器
 * 
 * 直接获取动力学真值，无噪声
 * 
 * 配置项：
 * - frequency_hz: 采样频率 (Hz)，默认 100.0
 */
class IdealImu : public core::ComponentBase, 
                 public interfaces::IImuSensor,
                 public core::IDependencyDeclarer,
                 public interfaces::IObservable {
public:
    IdealImu() : ComponentBase("IdealImu") {
        setExecutionFrequency(100.0);  // 默认 100 Hz
    }
    
    // --- 配置 ---
    
    void configure(const core::ConfigNode& config) override {
        if (config.isNull()) {
            return;
        }
        const auto& frequency_hz = config["frequency_hz"];
        if (!frequency_hz.isNull()) {
            setExecutionFrequency(frequency_hz.asDouble(100.0));
            return;
        }
        const auto& frequency = config["frequency"];
        if (!frequency.isNull()) {
            setExecutionFrequency(frequency.asDouble(100.0));
        }
    }
    
    // --- IImuSensor 接口实现 ---
    
    const interfaces::ImuData& getImuData() const override {
        return imu_data_;
    }
    
    bool isHealthy() const override {
        return true;
    }

    std::vector<core::DependencyDeclaration> getDependencies() const override {
        return {
            {std::type_index(typeid(interfaces::IDynamics)), "a dynamics interface", true}
        };
    }
    
    // --- ComponentBase 生命周期 ---
    
    void injectDependencies(core::ScopedRegistry& registry) override {
        dynamics_ = registry.getByName<interfaces::IDynamics>("dynamics");
    }
    
    void update(double dt) override {
        (void)dt;
        
        if (dynamics_) {
            const auto& state = dynamics_->getVehicleState();
            // 理想情况：直接使用真值
            // TODO: 实现真正的加速度计算
            imu_data_.acceleration = Vector3d::Zero();  // 占位
            imu_data_.angular_velocity = state.angular_velocity;
            imu_data_.timestamp = state.timestamp;
        }
    }

    std::vector<interfaces::ObservableField> getObservableFields() const override {
        core::ObservableFieldBuilder builder;
        builder.addVector3d("acceleration", [this]() -> const Vector3d& { return imu_data_.acceleration; });
        builder.addVector3d("angular_velocity", [this]() -> const Vector3d& { return imu_data_.angular_velocity; });
        builder.addScalar("timestamp", [this]() { return imu_data_.timestamp; });
        return builder.build();
    }
    
private:
    interfaces::ImuData imu_data_;
    interfaces::IDynamics* dynamics_ = nullptr;
};

// 自动注册到工厂
GNC_REGISTER_COMPONENT(IdealImu, interfaces::IImuSensor)

} // namespace gnc::components
