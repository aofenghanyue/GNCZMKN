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
#include "gnc/core/dependency_validator.hpp"
#include "gnc/core/observable_helpers.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/interfaces/dynamics/i_dynamics_model.hpp"
#include "gnc/interfaces/infrastructure/i_observable.hpp"
#include "gnc/interfaces/state/i_position_provider.hpp"
#include "gnc/interfaces/state/i_velocity_provider.hpp"

namespace gnc::components {

class TruthState : public core::ComponentBase,
                   public interfaces::IPositionProvider,
                   public interfaces::IVelocityProvider,
                   public interfaces::IObservable,
                   public core::IDependencyDeclarer {
public:
    TruthState() : ComponentBase("TruthState") {}

    void injectDependencies(core::ScopedRegistry& registry) override {
        dynamics_ = registry.getByName<interfaces::IDynamicsModel>("dynamics");
        position_provider_ = registry.getByName<interfaces::IPositionProvider>("dynamics");
        velocity_provider_ = registry.getByName<interfaces::IVelocityProvider>("dynamics");
    }

    void update(double) override {
        if (position_provider_) {
            position_ = position_provider_->getPosition();
        }
        if (velocity_provider_) {
            velocity_ = velocity_provider_->getVelocity();
        }
        if (!dynamics_) {
            return;
        }

        const auto& layout = dynamics_->getStateLayout();
        const auto& state = dynamics_->getState();
        if (layout.has("q_w") && layout.has("q_x") && layout.has("q_y") && layout.has("q_z")) {
            attitude_ = {
                state[layout.indexOf("q_w")],
                state[layout.indexOf("q_x")],
                state[layout.indexOf("q_y")],
                state[layout.indexOf("q_z")]
            };
        }
        if (layout.has("omega_x") && layout.has("omega_y") && layout.has("omega_z")) {
            angular_velocity_ = {
                state[layout.indexOf("omega_x")],
                state[layout.indexOf("omega_y")],
                state[layout.indexOf("omega_z")]
            };
        }
    }

    gnc::Vector3d getPosition() const override { return position_; }
    gnc::Vector3d getVelocity() const override { return velocity_; }

    std::vector<core::DependencyDeclaration> getDependencies() const override {
        return {
            {std::type_index(typeid(interfaces::IDynamicsModel)), "a dynamics model interface", true}
        };
    }

    std::vector<interfaces::ObservableField> getObservableFields() const override {
        core::ObservableFieldBuilder builder;
        builder.addVector3d("position", [this]() -> const gnc::Vector3d& { return position_; });
        builder.addVector3d("velocity", [this]() -> const gnc::Vector3d& { return velocity_; });
        builder.addScalar("speed", [this]() { return velocity_.norm(); });
        builder.addScalar("timestamp", [this]() { return getSimTime(); });
        return builder.build();
    }

private:
    interfaces::IDynamicsModel* dynamics_ = nullptr;
    interfaces::IPositionProvider* position_provider_ = nullptr;
    interfaces::IVelocityProvider* velocity_provider_ = nullptr;
    gnc::Vector3d position_ = gnc::Vector3d::Zero();
    gnc::Vector3d velocity_ = gnc::Vector3d::Zero();
    gnc::Quaterniond attitude_ = gnc::Quaterniond::Identity();
    gnc::Vector3d angular_velocity_ = gnc::Vector3d::Zero();
};

GNC_REGISTER_COMPONENT(TruthState,
                       interfaces::IPositionProvider,
                       interfaces::IVelocityProvider)

} // namespace gnc::components
