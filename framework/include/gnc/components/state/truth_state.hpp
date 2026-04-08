/**
 * @file truth_state.hpp
 * @brief 真实飞行器状态组件
 * 
 * 维护飞行器的真实物理状态（仿真真值）
 * 用于传感器误差建模、环境计算等
 */
#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/observable_helpers.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/interfaces/infrastructure/i_observable.hpp"
#include "gnc/interfaces/state/i_angular_velocity_provider.hpp"
#include "gnc/interfaces/state/i_attitude_provider.hpp"
#include "gnc/interfaces/state/i_position_provider.hpp"
#include "gnc/interfaces/state/i_velocity_provider.hpp"

namespace gnc::components {

class TruthState : public core::ComponentBase,
                   public interfaces::IPositionProvider,
                   public interfaces::IVelocityProvider,
                   public interfaces::IAttitudeProvider,
                   public interfaces::IAngularVelocityProvider,
                   public interfaces::IObservable {
public:
    TruthState() : ComponentBase("TruthState") {}

    void injectDependencies(core::ScopedRegistry& registry) override {
        registry.bindAll(
            core::bind(position_provider_, "dynamics"),
            core::bind(velocity_provider_, "dynamics"),
            core::bindIfPresent(attitude_provider_, "dynamics"),
            core::bindIfPresent(angular_velocity_provider_, "dynamics"));
    }

    void update(double) override {
        if (position_provider_) {
            position_ = position_provider_->getPosition();
        }
        if (velocity_provider_) {
            velocity_ = velocity_provider_->getVelocity();
        }
        attitude_ = attitude_provider_ ? attitude_provider_->getAttitude()
                                       : gnc::Quaterniond::Identity();
        angular_velocity_ = angular_velocity_provider_
                              ? angular_velocity_provider_->getAngularVelocity()
                              : gnc::Vector3d::Zero();
    }

    gnc::Vector3d getPosition() const override { return position_; }
    gnc::Vector3d getVelocity() const override { return velocity_; }
    gnc::Quaterniond getAttitude() const override { return attitude_; }
    gnc::Vector3d getAngularVelocity() const override { return angular_velocity_; }

    std::vector<interfaces::ObservableField> getObservableFields() const override {
        core::ObservableFieldBuilder builder;
        builder.addVector3d("position", [this]() -> const gnc::Vector3d& { return position_; });
        builder.addVector3d("velocity", [this]() -> const gnc::Vector3d& { return velocity_; });
        builder.addScalar("speed", [this]() { return velocity_.norm(); });
        builder.addScalar("timestamp", [this]() { return getSimTime(); });
        return builder.build();
    }

private:
    interfaces::IPositionProvider* position_provider_ = nullptr;
    interfaces::IVelocityProvider* velocity_provider_ = nullptr;
    interfaces::IAttitudeProvider* attitude_provider_ = nullptr;
    interfaces::IAngularVelocityProvider* angular_velocity_provider_ = nullptr;
    gnc::Vector3d position_ = gnc::Vector3d::Zero();
    gnc::Vector3d velocity_ = gnc::Vector3d::Zero();
    gnc::Quaterniond attitude_ = gnc::Quaterniond::Identity();
    gnc::Vector3d angular_velocity_ = gnc::Vector3d::Zero();
};

GNC_REGISTER_STARTER_COMPONENT(TruthState,
                       interfaces::IPositionProvider,
                       interfaces::IVelocityProvider,
                       interfaces::IAttitudeProvider,
                       interfaces::IAngularVelocityProvider)

} // namespace gnc::components
