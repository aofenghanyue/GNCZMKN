#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/core/state_layout.hpp"
#include "gnc/forms/cartesian_6dof/interfaces/i_input_provider.hpp"
#include "gnc/forms/cartesian_6dof/interfaces/i_truth_view.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_continuous_system.hpp"
#include "gnc/interfaces/i_observable.hpp"

#include <string>

namespace gnc::forms::cartesian_6dof {

class RigidBody final : public gnc::core::ComponentBase,
                        public gnc::interfaces::IContinuousSystem,
                        public ITruthView,
                        public gnc::interfaces::IObservable {
public:
    RigidBody() : ComponentBase("Cartesian6DoFRigidBody") {
        position_x_index_ = layout_.addVariable("position_x");
        position_y_index_ = layout_.addVariable("position_y");
        position_z_index_ = layout_.addVariable("position_z");
        velocity_x_index_ = layout_.addVariable("velocity_x");
        velocity_y_index_ = layout_.addVariable("velocity_y");
        velocity_z_index_ = layout_.addVariable("velocity_z");
        q_w_index_ = layout_.addVariable("q_body_to_frame_w");
        q_x_index_ = layout_.addVariable("q_body_to_frame_x");
        q_y_index_ = layout_.addVariable("q_body_to_frame_y");
        q_z_index_ = layout_.addVariable("q_body_to_frame_z");
        roll_rate_index_ = layout_.addVariable("roll_rate_body_radps");
        pitch_rate_index_ = layout_.addVariable("pitch_rate_body_radps");
        yaw_rate_index_ = layout_.addVariable("yaw_rate_body_radps");
        state_vector_ = Eigen::VectorXd::Zero(layout_.dimension());
        initial_state_vector_ = state_vector_;
        packState(State{}, initial_state_vector_);
        state_vector_ = initial_state_vector_;
    }

    void configure(const gnc::core::ConfigNode& config) override {
        configure(config, "config");
    }

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        input_lookup_name_ =
            reader.optionalString("input_lookup_name", input_lookup_name_);

        const auto initial_state_reader = reader.requiredObject("initial_state");
        State initial;
        const auto position =
            initial_state_reader.requiredDoubleArray("position_m", 3);
        initial.position_m =
            gnc::math::Vector3(position[0], position[1], position[2]);

        const auto velocity =
            initial_state_reader.requiredDoubleArray("velocity_mps", 3);
        initial.velocity_mps =
            gnc::math::Vector3(velocity[0], velocity[1], velocity[2]);

        const auto attitude =
            initial_state_reader.requiredDoubleArray("attitude_body_to_frame", 4);
        initial.attitude_body_to_frame = Eigen::Quaterniond(
            attitude[0], attitude[1], attitude[2], attitude[3]);
        normalize(initial.attitude_body_to_frame);

        const auto angular_rate =
            initial_state_reader.requiredDoubleArray("angular_rate_body_radps", 3);
        initial.angular_rate_body_radps =
            gnc::math::Vector3(angular_rate[0], angular_rate[1], angular_rate[2]);
        initial_state_reader.validateNoUnknownKeys();

        packState(initial, initial_state_vector_);
        state_vector_ = initial_state_vector_;
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(gnc::core::bind(input_provider_, input_lookup_name_));
    }

    void initialize() override { publish(getSimTime()); }

    void publish(double time) override { truth_ = sampleTruth(state_vector_, time); }

    void update(double) override {}

    const gnc::core::StateLayout& getStateLayout() const override {
        return layout_;
    }

    void computeDerivatives(double,
                            const Eigen::VectorXd&,
                            Eigen::VectorXd& derivative_vector) const override {
        derivative_vector = Eigen::VectorXd::Zero(layout_.dimension());
    }

    const Eigen::VectorXd& getState() const override { return state_vector_; }

    void setState(const Eigen::VectorXd& state) override {
        state_vector_ = state;
        normalizeStateQuaternion(state_vector_);
    }

    Eigen::VectorXd getInitialState() const override {
        return initial_state_vector_;
    }

    const Truth& getCartesian6DoFTruth() const override { return truth_; }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addVector3("position", [this]() -> const gnc::math::Vector3& {
            return truth_.state.position_m;
        });
        builder.addVector3("velocity", [this]() -> const gnc::math::Vector3& {
            return truth_.state.velocity_mps;
        });
        builder.addQuaternion("attitude_body_to_frame",
                              [this]() -> const Eigen::Quaterniond& {
                                  return truth_.state.attitude_body_to_frame;
                              });
        builder.addVector3("angular_rate_body",
                           [this]() -> const gnc::math::Vector3& {
                               return truth_.state.angular_rate_body_radps;
                           });
        builder.addVector3("acceleration", [this]() -> const gnc::math::Vector3& {
            return truth_.acceleration_mps2;
        });
        builder.addVector3("angular_acceleration_body",
                           [this]() -> const gnc::math::Vector3& {
                               return truth_.angular_acceleration_body_radps2;
                           });
        builder.addScalar("speed", [this]() {
            return truth_.state.velocity_mps.norm();
        });
        builder.addScalar("altitude", [this]() {
            return truth_.state.position_m.z();
        });
        return builder.build();
    }

private:
    State unpackState(const Eigen::VectorXd& state_vector) const {
        State state;
        state.position_m =
            gnc::math::Vector3(state_vector[position_x_index_],
                               state_vector[position_y_index_],
                               state_vector[position_z_index_]);
        state.velocity_mps =
            gnc::math::Vector3(state_vector[velocity_x_index_],
                               state_vector[velocity_y_index_],
                               state_vector[velocity_z_index_]);
        state.attitude_body_to_frame =
            Eigen::Quaterniond(state_vector[q_w_index_],
                               state_vector[q_x_index_],
                               state_vector[q_y_index_],
                               state_vector[q_z_index_]);
        normalize(state.attitude_body_to_frame);
        state.angular_rate_body_radps =
            gnc::math::Vector3(state_vector[roll_rate_index_],
                               state_vector[pitch_rate_index_],
                               state_vector[yaw_rate_index_]);
        return state;
    }

    void packState(const State& state, Eigen::VectorXd& state_vector) const {
        Eigen::Quaterniond attitude = state.attitude_body_to_frame;
        normalize(attitude);
        state_vector = Eigen::VectorXd::Zero(layout_.dimension());
        state_vector[position_x_index_] = state.position_m.x();
        state_vector[position_y_index_] = state.position_m.y();
        state_vector[position_z_index_] = state.position_m.z();
        state_vector[velocity_x_index_] = state.velocity_mps.x();
        state_vector[velocity_y_index_] = state.velocity_mps.y();
        state_vector[velocity_z_index_] = state.velocity_mps.z();
        state_vector[q_w_index_] = attitude.w();
        state_vector[q_x_index_] = attitude.x();
        state_vector[q_y_index_] = attitude.y();
        state_vector[q_z_index_] = attitude.z();
        state_vector[roll_rate_index_] = state.angular_rate_body_radps.x();
        state_vector[pitch_rate_index_] = state.angular_rate_body_radps.y();
        state_vector[yaw_rate_index_] = state.angular_rate_body_radps.z();
    }

    Truth sampleTruth(const Eigen::VectorXd& state_vector, double time) const {
        Truth truth;
        truth.state = unpackState(state_vector);
        truth.sample_time_s = time;
        const Input input = resolveInput(truth, time);
        truth.acceleration_mps2 = input.acceleration_mps2;
        truth.angular_acceleration_body_radps2 =
            input.angular_acceleration_body_radps2;
        return truth;
    }

    Input resolveInput(const Truth& truth, double time) const {
        if (!input_provider_) {
            return {};
        }
        return input_provider_->computeCartesian6DoFInput(truth, time);
    }

    static void normalize(Eigen::Quaterniond& q) {
        if (q.norm() == 0.0) {
            q = Eigen::Quaterniond::Identity();
        } else {
            q.normalize();
        }
    }

    void normalizeStateQuaternion(Eigen::VectorXd& state_vector) const {
        Eigen::Quaterniond q(state_vector[q_w_index_],
                             state_vector[q_x_index_],
                             state_vector[q_y_index_],
                             state_vector[q_z_index_]);
        normalize(q);
        state_vector[q_w_index_] = q.w();
        state_vector[q_x_index_] = q.x();
        state_vector[q_y_index_] = q.y();
        state_vector[q_z_index_] = q.z();
    }

    gnc::core::StateLayout layout_;
    Eigen::VectorXd state_vector_;
    Eigen::VectorXd initial_state_vector_;
    IInputProvider* input_provider_ = nullptr;
    std::string input_lookup_name_ = "interaction";
    Truth truth_{};
    int position_x_index_ = -1;
    int position_y_index_ = -1;
    int position_z_index_ = -1;
    int velocity_x_index_ = -1;
    int velocity_y_index_ = -1;
    int velocity_z_index_ = -1;
    int q_w_index_ = -1;
    int q_x_index_ = -1;
    int q_y_index_ = -1;
    int q_z_index_ = -1;
    int roll_rate_index_ = -1;
    int pitch_rate_index_ = -1;
    int yaw_rate_index_ = -1;
};

} // namespace gnc::forms::cartesian_6dof
