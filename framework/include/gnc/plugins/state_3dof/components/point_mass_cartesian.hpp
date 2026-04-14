#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/state_layout.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_continuous_system.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/plugins/state_3dof/interfaces/i_state_solver_3dof.hpp"

namespace gnc::plugins::state_3dof {

class PointMassCartesian final : public gnc::core::ComponentBase,
                                 public gnc::interfaces::IContinuousSystem,
                                 public IStateSolver3DOF,
                                 public gnc::interfaces::IObservable {
public:
    PointMassCartesian() : ComponentBase("PointMassCartesian") {
        position_x_index_ = layout_.addVariable("position_x");
        position_y_index_ = layout_.addVariable("position_y");
        position_z_index_ = layout_.addVariable("position_z");
        velocity_x_index_ = layout_.addVariable("velocity_x");
        velocity_y_index_ = layout_.addVariable("velocity_y");
        velocity_z_index_ = layout_.addVariable("velocity_z");
        state_ = Eigen::VectorXd::Zero(layout_.dimension());
        initial_state_ = state_;
    }

    void configure(const gnc::core::ConfigNode& config) override {
        const auto& initial_position = config["initial_position"];
        if (initial_position.isArray() && initial_position.size() >= 3) {
            initial_state_[position_x_index_] = initial_position[0].asDouble(0.0);
            initial_state_[position_y_index_] = initial_position[1].asDouble(0.0);
            initial_state_[position_z_index_] = initial_position[2].asDouble(0.0);
        }

        const auto& initial_velocity = config["initial_velocity"];
        if (initial_velocity.isArray() && initial_velocity.size() >= 3) {
            initial_state_[velocity_x_index_] = initial_velocity[0].asDouble(0.0);
            initial_state_[velocity_y_index_] = initial_velocity[1].asDouble(0.0);
            initial_state_[velocity_z_index_] = initial_velocity[2].asDouble(0.0);
        }

        const auto& constant_acceleration = config["constant_acceleration"];
        if (constant_acceleration.isArray() && constant_acceleration.size() >= 3) {
            constant_acceleration_ = gnc::math::Vector3(
                constant_acceleration[0].asDouble(0.0),
                constant_acceleration[1].asDouble(0.0),
                constant_acceleration[2].asDouble(0.0));
        }

        mass_kg_ = config["mass_kg"].asDouble(mass_kg_);
        state_ = initial_state_;
    }

    const gnc::core::StateLayout& getStateLayout() const override { return layout_; }

    void computeDerivatives(double,
                            const Eigen::VectorXd& state,
                            Eigen::VectorXd& derivative) const override {
        derivative = Eigen::VectorXd::Zero(layout_.dimension());
        derivative[position_x_index_] = state[velocity_x_index_];
        derivative[position_y_index_] = state[velocity_y_index_];
        derivative[position_z_index_] = state[velocity_z_index_];
        derivative[velocity_x_index_] = constant_acceleration_.x();
        derivative[velocity_y_index_] = constant_acceleration_.y();
        derivative[velocity_z_index_] = constant_acceleration_.z();
    }

    const Eigen::VectorXd& getState() const override { return state_; }
    void setState(const Eigen::VectorXd& state) override { state_ = state; }
    Eigen::VectorXd getInitialState() const override { return initial_state_; }
    void update(double) override {}

    gnc::math::Vector3 getPosition() const override {
        return gnc::math::Vector3(state_[position_x_index_],
                                  state_[position_y_index_],
                                  state_[position_z_index_]);
    }

    gnc::math::Vector3 getVelocity() const override {
        return gnc::math::Vector3(state_[velocity_x_index_],
                                  state_[velocity_y_index_],
                                  state_[velocity_z_index_]);
    }

    double getMass() const override { return mass_kg_; }
    double getSpeed() const override { return getVelocity().norm(); }
    double getAltitude() const override { return getPosition().z(); }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addVector3("position", [this]() -> const gnc::math::Vector3& {
            position_cache_ = getPosition();
            return position_cache_;
        });
        builder.addVector3("velocity", [this]() -> const gnc::math::Vector3& {
            velocity_cache_ = getVelocity();
            return velocity_cache_;
        });
        builder.addScalar("speed", [this]() { return getSpeed(); });
        builder.addScalar("altitude", [this]() { return getAltitude(); });
        builder.addScalar("mass", [this]() { return getMass(); });
        return builder.build();
    }

private:
    gnc::core::StateLayout layout_;
    Eigen::VectorXd state_;
    Eigen::VectorXd initial_state_;
    gnc::math::Vector3 constant_acceleration_ = gnc::math::Vector3::Zero();
    double mass_kg_ = 100.0;
    int position_x_index_ = -1;
    int position_y_index_ = -1;
    int position_z_index_ = -1;
    int velocity_x_index_ = -1;
    int velocity_y_index_ = -1;
    int velocity_z_index_ = -1;
    mutable gnc::math::Vector3 position_cache_ = gnc::math::Vector3::Zero();
    mutable gnc::math::Vector3 velocity_cache_ = gnc::math::Vector3::Zero();
};

} // namespace gnc::plugins::state_3dof
