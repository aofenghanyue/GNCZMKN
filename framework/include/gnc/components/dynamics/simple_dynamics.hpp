/**
 * @file simple_dynamics.hpp
 * @brief 简单动力学模型
 */
#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/core/dependency_validator.hpp"
#include "gnc/core/observable_helpers.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/core/state_layout.hpp"
#include "gnc/interfaces/dynamics/i_dynamics_model.hpp"
#include "gnc/interfaces/gnc/i_guidance_6dof.hpp"
#include "gnc/interfaces/infrastructure/i_observable.hpp"
#include "gnc/interfaces/state/i_position_provider.hpp"
#include "gnc/interfaces/state/i_velocity_provider.hpp"

namespace gnc::components {

/**
 * @brief 简单六自由度动力学
 * 
 * 占位实现，展示框架用法
 */
class SimpleDynamics : public core::ComponentBase,
                       public interfaces::IDynamicsModel,
                       public interfaces::IPositionProvider,
                       public interfaces::IVelocityProvider,
                       public interfaces::IObservable,
                       public core::IDependencyDeclarer {
public:
    SimpleDynamics() : ComponentBase("SimpleDynamics") {
        setExecutionFrequency(0.0);
        idx_px_ = layout_.addVariable("pos_x");
        idx_py_ = layout_.addVariable("pos_y");
        idx_pz_ = layout_.addVariable("pos_z");
        idx_vx_ = layout_.addVariable("vel_x");
        idx_vy_ = layout_.addVariable("vel_y");
        idx_vz_ = layout_.addVariable("vel_z");
        state_ = Eigen::VectorXd::Zero(layout_.dimension());
        initial_state_ = state_;
    }

    const core::StateLayout& getStateLayout() const override {
        return layout_;
    }

    void computeDerivatives(double,
                            const Eigen::VectorXd& x,
                            Eigen::VectorXd& dxdt) const override {
        dxdt = Eigen::VectorXd::Zero(layout_.dimension());

        gnc::Vector3d acceleration = gnc::Vector3d::Zero();
        if (guidance_ && guidance_->isActive()) {
            acceleration = guidance_->getGuidanceCommand().acceleration_cmd;
        }

        dxdt[idx_px_] = x[idx_vx_];
        dxdt[idx_py_] = x[idx_vy_];
        dxdt[idx_pz_] = x[idx_vz_];
        dxdt[idx_vx_] = acceleration.x;
        dxdt[idx_vy_] = acceleration.y;
        dxdt[idx_vz_] = acceleration.z;
    }

    const Eigen::VectorXd& getState() const override {
        return state_;
    }

    void setState(const Eigen::VectorXd& x) override {
        state_ = x;
    }

    Eigen::VectorXd getInitialState() const override {
        return initial_state_;
    }

    gnc::Vector3d getPosition() const override {
        return {state_[idx_px_], state_[idx_py_], state_[idx_pz_]};
    }

    gnc::Vector3d getVelocity() const override {
        return {state_[idx_vx_], state_[idx_vy_], state_[idx_vz_]};
    }

    void configure(const core::ConfigNode& config) override {
        if (config.isNull()) {
            return;
        }
        const auto& initial_position = config["initial_position"];
        if (initial_position.isArray() && initial_position.size() >= 3) {
            initial_state_[idx_px_] = initial_position[0].asDouble(0.0);
            initial_state_[idx_py_] = initial_position[1].asDouble(0.0);
            initial_state_[idx_pz_] = initial_position[2].asDouble(0.0);
        }
        const auto& initial_velocity = config["initial_velocity"];
        if (initial_velocity.isArray() && initial_velocity.size() >= 3) {
            initial_state_[idx_vx_] = initial_velocity[0].asDouble(0.0);
            initial_state_[idx_vy_] = initial_velocity[1].asDouble(0.0);
            initial_state_[idx_vz_] = initial_velocity[2].asDouble(0.0);
        }
        state_ = initial_state_;
    }

    void injectDependencies(core::ScopedRegistry& registry) override {
        guidance_ = registry.getByName<interfaces::IGuidance6DOF>("guidance");
    }

    void update(double) override {
    }

    std::vector<core::DependencyDeclaration> getDependencies() const override {
        return {};
    }

    std::vector<interfaces::ObservableField> getObservableFields() const override {
        core::ObservableFieldBuilder builder;
        builder.addVector3d("position", [this]() -> const gnc::Vector3d& {
            position_cache_ = getPosition();
            return position_cache_;
        });
        builder.addVector3d("velocity", [this]() -> const gnc::Vector3d& {
            velocity_cache_ = getVelocity();
            return velocity_cache_;
        });
        builder.addScalar("speed", [this]() { return getVelocity().norm(); });
        builder.addScalar("timestamp", [this]() { return getSimTime(); });
        return builder.build();
    }

private:
    core::StateLayout layout_;
    Eigen::VectorXd state_;
    Eigen::VectorXd initial_state_;
    interfaces::IGuidance6DOF* guidance_ = nullptr;
    int idx_px_ = -1;
    int idx_py_ = -1;
    int idx_pz_ = -1;
    int idx_vx_ = -1;
    int idx_vy_ = -1;
    int idx_vz_ = -1;
    mutable gnc::Vector3d position_cache_ = gnc::Vector3d::Zero();
    mutable gnc::Vector3d velocity_cache_ = gnc::Vector3d::Zero();
};

GNC_REGISTER_COMPONENT(SimpleDynamics,
                       interfaces::IDynamicsModel,
                       interfaces::IPositionProvider,
                       interfaces::IVelocityProvider)

} // namespace gnc::components
