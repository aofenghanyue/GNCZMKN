/**
 * @file ideal_imu.hpp
 * @brief 理想 IMU 传感器实现
 */
#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/core/observable_helpers.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/interfaces/infrastructure/i_observable.hpp"
#include "gnc/interfaces/sensors/i_imu_sensor.hpp"
#include "gnc/interfaces/state/i_angular_velocity_provider.hpp"

namespace gnc::components {

/**
 * @brief 理想 IMU 传感器
 *
 * 当前 starter 版本不做加速度重建；若作用域内存在角速度 provider，
 * 则直接读取真值角速度，否则输出零向量。
 */
class IdealImu : public core::ComponentBase,
                 public interfaces::IImuSensor,
                 public interfaces::IObservable {
public:
    IdealImu() : ComponentBase("IdealImu") {
        setExecutionFrequency(100.0);
    }

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

    const interfaces::ImuData& getImuData() const override {
        return imu_data_;
    }

    bool isHealthy() const override {
        return true;
    }

    void injectDependencies(core::ScopedRegistry& registry) override {
        registry.bindAll(core::bindIfPresent(angular_velocity_provider_, "dynamics"));
    }

    void update(double) override {
        imu_data_.acceleration = Vector3d::Zero();
        imu_data_.angular_velocity = angular_velocity_provider_
                                       ? angular_velocity_provider_->getAngularVelocity()
                                       : Vector3d::Zero();
        imu_data_.timestamp = getSimTime();
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
    interfaces::IAngularVelocityProvider* angular_velocity_provider_ = nullptr;
};

GNC_REGISTER_STARTER_COMPONENT(IdealImu, interfaces::IImuSensor)

} // namespace gnc::components
