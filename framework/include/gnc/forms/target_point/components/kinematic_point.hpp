#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/state_layout.hpp"
#include "gnc/forms/target_point/interfaces/i_truth_view.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_continuous_system.hpp"
#include "gnc/interfaces/i_observable.hpp"

namespace gnc::forms::target_point {

class KinematicPoint final : public gnc::core::ComponentBase,
                             public gnc::interfaces::IContinuousSystem,
                             public ITruthView,
                             public gnc::interfaces::IObservable {
public:
    KinematicPoint() : ComponentBase("TargetPointKinematic") {
        position_x_index_ = layout_.addVariable("position_ecef_x_m");
        position_y_index_ = layout_.addVariable("position_ecef_y_m");
        position_z_index_ = layout_.addVariable("position_ecef_z_m");
        velocity_x_index_ = layout_.addVariable("velocity_ecef_x_mps");
        velocity_y_index_ = layout_.addVariable("velocity_ecef_y_mps");
        velocity_z_index_ = layout_.addVariable("velocity_ecef_z_mps");
        state_vector_ = Eigen::VectorXd::Zero(layout_.dimension());
        initial_state_vector_ = state_vector_;
    }

    void configure(const gnc::core::ConfigNode& config) override {
        configure(config, "config");
    }

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        const auto position =
            reader.requiredDoubleArray("initial_position_ecef_m", 3);
        const auto velocity = reader.requiredDoubleArray("velocity_ecef_mps", 3);
        State initial;
        initial.position_ecef_m =
            gnc::math::Vector3(position[0], position[1], position[2]);
        initial.velocity_ecef_mps =
            gnc::math::Vector3(velocity[0], velocity[1], velocity[2]);
        packState(initial, initial_state_vector_);
        state_vector_ = initial_state_vector_;
        reader.validateNoUnknownKeys();
    }

    void initialize() override { publish(getSimTime()); }

    void publish(double time) override {
        truth_ = sampleTruth(state_vector_, time);
    }

    void update(double) override {}

    const gnc::core::StateLayout& getStateLayout() const override {
        return layout_;
    }

    void computeDerivatives(double,
                            const Eigen::VectorXd& state_vector,
                            Eigen::VectorXd& derivative_vector) const override {
        derivative_vector = Eigen::VectorXd::Zero(layout_.dimension());
        derivative_vector[position_x_index_] = state_vector[velocity_x_index_];
        derivative_vector[position_y_index_] = state_vector[velocity_y_index_];
        derivative_vector[position_z_index_] = state_vector[velocity_z_index_];
    }

    const Eigen::VectorXd& getState() const override { return state_vector_; }

    void setState(const Eigen::VectorXd& state) override { state_vector_ = state; }

    Eigen::VectorXd getInitialState() const override {
        return initial_state_vector_;
    }

    const Truth& getTargetPointTruth() const override { return truth_; }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addVector3("position_ecef", [this]() -> const gnc::math::Vector3& {
            return truth_.position_ecef_m;
        });
        builder.addVector3("velocity_ecef", [this]() -> const gnc::math::Vector3& {
            return truth_.velocity_ecef_mps;
        });
        return builder.build();
    }

private:
    State unpackState(const Eigen::VectorXd& state_vector) const {
        State state;
        state.position_ecef_m = gnc::math::Vector3(
            state_vector[position_x_index_],
            state_vector[position_y_index_],
            state_vector[position_z_index_]);
        state.velocity_ecef_mps = gnc::math::Vector3(
            state_vector[velocity_x_index_],
            state_vector[velocity_y_index_],
            state_vector[velocity_z_index_]);
        return state;
    }

    void packState(const State& state, Eigen::VectorXd& state_vector) const {
        state_vector = Eigen::VectorXd::Zero(layout_.dimension());
        state_vector[position_x_index_] = state.position_ecef_m.x();
        state_vector[position_y_index_] = state.position_ecef_m.y();
        state_vector[position_z_index_] = state.position_ecef_m.z();
        state_vector[velocity_x_index_] = state.velocity_ecef_mps.x();
        state_vector[velocity_y_index_] = state.velocity_ecef_mps.y();
        state_vector[velocity_z_index_] = state.velocity_ecef_mps.z();
    }

    Truth sampleTruth(const Eigen::VectorXd& state_vector, double time) const {
        Truth truth;
        truth.state = unpackState(state_vector);
        truth.position_ecef_m = truth.state.position_ecef_m;
        truth.velocity_ecef_mps = truth.state.velocity_ecef_mps;
        truth.sample_time_s = time;
        return truth;
    }

    gnc::core::StateLayout layout_;
    Eigen::VectorXd state_vector_;
    Eigen::VectorXd initial_state_vector_;
    Truth truth_{};
    int position_x_index_ = -1;
    int position_y_index_ = -1;
    int position_z_index_ = -1;
    int velocity_x_index_ = -1;
    int velocity_y_index_ = -1;
    int velocity_z_index_ = -1;
};

} // namespace gnc::forms::target_point
