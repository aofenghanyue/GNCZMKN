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
#include "gnc/core/state_layout.hpp"
#include "gnc/interfaces/dynamics/i_dynamics_model.hpp"
#include "gnc/interfaces/state/i_velocity_provider.hpp"
#include "gnc/interfaces/sensors/i_imu_sensor.hpp"
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
            {std::type_index(typeid(interfaces::IDynamicsModel)), "a dynamics model interface", true}
        };
    }
    
    // --- ComponentBase 生命周期 ---
    
    void injectDependencies(core::ScopedRegistry& registry) override {
        dynamics_ = registry.getByName<interfaces::IDynamicsModel>("dynamics");
    }
    
    void update(double dt) override {
        (void)dt;
        
        if (!dynamics_) {
            return;
        }

        const auto& layout = dynamics_->getStateLayout();
        const auto& state = dynamics_->getState();
        imu_data_.acceleration = Vector3d::Zero();
        imu_data_.angular_velocity = Vector3d::Zero();
        imu_data_.timestamp = getSimTime();

        const int omega_x = layout.indexOf("omega_x");
        const int omega_y = layout.indexOf("omega_y");
        const int omega_z = layout.indexOf("omega_z");
        if (omega_x >= 0 && omega_y >= 0 && omega_z >= 0) {
            imu_data_.angular_velocity = {
                state[omega_x],
                state[omega_y],
                state[omega_z]
            };
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
    interfaces::IDynamicsModel* dynamics_ = nullptr;
};

// 自动注册到工厂
GNC_REGISTER_COMPONENT(IdealImu, interfaces::IImuSensor)

} // namespace gnc::components
