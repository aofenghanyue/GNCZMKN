#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/core/state_layout.hpp"
#include "gnc/forms/cartesian_3dof/interfaces/i_input_provider.hpp"
#include "gnc/forms/cartesian_3dof/interfaces/i_truth_view.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_continuous_system.hpp"
#include "gnc/interfaces/i_observable.hpp"

namespace gnc::forms::cartesian_3dof {

class PointMass final : public gnc::core::ComponentBase,
                        public gnc::interfaces::IContinuousSystem,
                        public ITruthView,
                        public gnc::interfaces::IObservable {
public:
    PointMass() : ComponentBase("Cartesian3DoFPointMass") {
        position_x_index_ = layout_.addVariable("position_x");
        position_y_index_ = layout_.addVariable("position_y");
        position_z_index_ = layout_.addVariable("position_z");
        velocity_x_index_ = layout_.addVariable("velocity_x");
        velocity_y_index_ = layout_.addVariable("velocity_y");
        velocity_z_index_ = layout_.addVariable("velocity_z");
        state_vector_ = Eigen::VectorXd::Zero(layout_.dimension());
        initial_state_vector_ = state_vector_;
    }

    void configure(const gnc::core::ConfigNode& config) override {
        const auto& initial_position = config["initial_position"];
        if (initial_position.isArray() && initial_position.size() >= 3) {
            initial_state_vector_[position_x_index_] = initial_position[0].asDouble(0.0);
            initial_state_vector_[position_y_index_] = initial_position[1].asDouble(0.0);
            initial_state_vector_[position_z_index_] = initial_position[2].asDouble(0.0);
        }

        const auto& initial_velocity = config["initial_velocity"];
        if (initial_velocity.isArray() && initial_velocity.size() >= 3) {
            initial_state_vector_[velocity_x_index_] = initial_velocity[0].asDouble(0.0);
            initial_state_vector_[velocity_y_index_] = initial_velocity[1].asDouble(0.0);
            initial_state_vector_[velocity_z_index_] = initial_velocity[2].asDouble(0.0);
        }

        input_lookup_name_ = config["input_lookup_name"].asString(input_lookup_name_);
        state_vector_ = initial_state_vector_;
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(gnc::core::bind(input_provider_, input_lookup_name_));
    }

    void initialize() override { refreshTruth(getSimTime()); }

    void update(double dt) override { refreshTruth(getSimTime() + dt); }

    const gnc::core::StateLayout& getStateLayout() const override { return layout_; }

    void computeDerivatives(double time,
                            const Eigen::VectorXd& state,
                            Eigen::VectorXd& derivative) const override {
        const State unpacked_state = unpackState(state);
        const Truth truth = buildTruth(unpacked_state, time);
        const Input input = resolveInput(truth, time);

        derivative = Eigen::VectorXd::Zero(layout_.dimension());
        derivative[position_x_index_] = unpacked_state.velocity_mps.x();
        derivative[position_y_index_] = unpacked_state.velocity_mps.y();
        derivative[position_z_index_] = unpacked_state.velocity_mps.z();
        derivative[velocity_x_index_] = input.acceleration_mps2.x();
        derivative[velocity_y_index_] = input.acceleration_mps2.y();
        derivative[velocity_z_index_] = input.acceleration_mps2.z();
    }

    const Eigen::VectorXd& getState() const override { return state_vector_; }
    void setState(const Eigen::VectorXd& state) override { state_vector_ = state; }
    Eigen::VectorXd getInitialState() const override { return initial_state_vector_; }

    gnc::math::Vector3 getPosition() const {
        return unpackState(state_vector_).position_m;
    }

    gnc::math::Vector3 getVelocity() const {
        return unpackState(state_vector_).velocity_mps;
    }

    double getSpeed() const { return getVelocity().norm(); }
    double getAltitude() const { return getPosition().z(); }

    const Truth& getCartesian3DoFTruth() const override { return truth_; }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addVector3("position", [this]() -> const gnc::math::Vector3& {
            return truth_.state.position_m;
        });
        builder.addVector3("velocity", [this]() -> const gnc::math::Vector3& {
            return truth_.state.velocity_mps;
        });
        builder.addVector3("acceleration", [this]() -> const gnc::math::Vector3& {
            return truth_.acceleration_mps2;
        });
        builder.addScalar("speed", [this]() { return getSpeed(); });
        builder.addScalar("altitude", [this]() { return getAltitude(); });
        return builder.build();
    }

private:
    State unpackState(const Eigen::VectorXd& state) const {
        State unpacked;
        unpacked.position_m =
            gnc::math::Vector3(state[position_x_index_],
                               state[position_y_index_],
                               state[position_z_index_]);
        unpacked.velocity_mps =
            gnc::math::Vector3(state[velocity_x_index_],
                               state[velocity_y_index_],
                               state[velocity_z_index_]);
        return unpacked;
    }

    Truth buildTruth(const State& state, double sample_time) const {
        Truth truth;
        truth.state = state;
        truth.sample_time_s = sample_time;
        return truth;
    }

    Input resolveInput(const Truth& truth, double time) const {
        if (!input_provider_) {
            return {};
        }
        return input_provider_->computeCartesian3DoFInput(truth, time);
    }

    void refreshTruth(double sample_time) {
        truth_ = buildTruth(unpackState(state_vector_), sample_time);
        truth_.acceleration_mps2 = resolveInput(truth_, sample_time).acceleration_mps2;
    }

    gnc::core::StateLayout layout_;
    Eigen::VectorXd state_vector_;
    Eigen::VectorXd initial_state_vector_;
    IInputProvider* input_provider_ = nullptr;
    std::string input_lookup_name_ = "interaction";
    int position_x_index_ = -1;
    int position_y_index_ = -1;
    int position_z_index_ = -1;
    int velocity_x_index_ = -1;
    int velocity_y_index_ = -1;
    int velocity_z_index_ = -1;
    Truth truth_{};
};

} // namespace gnc::forms::cartesian_3dof
