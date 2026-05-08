#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/core/state_layout.hpp"
#include "gnc/environment/interfaces/i_earth.hpp"
#include "gnc/forms/local_spherical_6dof/interfaces/i_input_provider.hpp"
#include "gnc/forms/local_spherical_6dof/interfaces/i_truth_view.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_continuous_system.hpp"
#include "gnc/interfaces/i_observable.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace gnc::forms::local_spherical_6dof {

class RigidBody final : public gnc::core::ComponentBase,
                        public gnc::interfaces::IContinuousSystem,
                        public ITruthView,
                        public gnc::interfaces::IObservable {
public:
    RigidBody() : ComponentBase("LocalSpherical6DoFRigidBody") {
        longitude_index_ = layout_.addVariable("longitude_rad");
        latitude_index_ = layout_.addVariable("latitude_rad");
        altitude_index_ = layout_.addVariable("altitude_m");
        velocity_north_index_ = layout_.addVariable("velocity_north_mps");
        velocity_up_index_ = layout_.addVariable("velocity_up_mps");
        velocity_east_index_ = layout_.addVariable("velocity_east_mps");
        q_w_index_ = layout_.addVariable("q_body_to_nue_w");
        q_x_index_ = layout_.addVariable("q_body_to_nue_x");
        q_y_index_ = layout_.addVariable("q_body_to_nue_y");
        q_z_index_ = layout_.addVariable("q_body_to_nue_z");
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
        earth_lookup_name_ = reader.optionalString("earth_lookup_name",
                                                   earth_lookup_name_);
        input_lookup_name_ = reader.optionalString("input_lookup_name",
                                                   input_lookup_name_);
        reference_radius_m_ = reader.optionalDouble("reference_radius_m",
                                                   reference_radius_m_);

        const auto initial_state_reader = reader.requiredObject("initial_state");
        State initial;
        initial.longitude_rad =
            initial_state_reader.requiredDouble("longitude_rad");
        initial.latitude_rad = initial_state_reader.requiredDouble("latitude_rad");
        initial.altitude_m = initial_state_reader.requiredDouble("altitude_m");

        const auto velocity =
            initial_state_reader.requiredDoubleArray("velocity_nue_mps", 3);
        initial.velocity_nue_mps =
            gnc::math::Vector3(velocity[0], velocity[1], velocity[2]);

        const auto attitude =
            initial_state_reader.requiredDoubleArray("attitude_body_to_nue", 4);
        initial.attitude_body_to_nue = Eigen::Quaterniond(
            attitude[0], attitude[1], attitude[2], attitude[3]);
        initial.attitude_body_to_nue.normalize();

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
        registry.bindAll(
            gnc::core::bindIfPresent(earth_, earth_lookup_name_),
            gnc::core::bind(input_provider_, input_lookup_name_));
    }

    void initialize() override { publish(getSimTime()); }

    void publish(double time) override { truth_ = sampleTruth(state_vector_, time); }

    void update(double) override {}

    const gnc::core::StateLayout& getStateLayout() const override {
        return layout_;
    }

    void computeDerivatives(double time,
                            const Eigen::VectorXd& state_vector,
                            Eigen::VectorXd& derivative_vector) const override {
        (void)state_vector;
        (void)time;
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

    const Truth& getLocalSpherical6DoFTruth() const override { return truth_; }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("longitude_rad",
                          [this]() { return truth_.state.longitude_rad; });
        builder.addScalar("latitude_rad",
                          [this]() { return truth_.state.latitude_rad; });
        builder.addScalar("altitude_m",
                          [this]() { return truth_.state.altitude_m; });
        builder.addVector3("velocity_nue", [this]() -> const gnc::math::Vector3& {
            return truth_.state.velocity_nue_mps;
        });
        builder.addQuaternion("attitude_body_to_nue",
                              [this]() -> const Eigen::Quaterniond& {
                                  return truth_.state.attitude_body_to_nue;
                              });
        builder.addVector3("angular_rate_body",
                           [this]() -> const gnc::math::Vector3& {
                               return truth_.state.angular_rate_body_radps;
                           });
        builder.addVector3("local_acceleration_nue",
                           [this]() -> const gnc::math::Vector3& {
                               return truth_.local_acceleration_nue_mps2;
                           });
        builder.addVector3("angular_acceleration_body",
                           [this]() -> const gnc::math::Vector3& {
                               return truth_.angular_acceleration_body_radps2;
                           });
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
        state.longitude_rad = state_vector[longitude_index_];
        state.latitude_rad = state_vector[latitude_index_];
        state.altitude_m = state_vector[altitude_index_];
        state.velocity_nue_mps = gnc::math::Vector3(
            state_vector[velocity_north_index_],
            state_vector[velocity_up_index_],
            state_vector[velocity_east_index_]);
        state.attitude_body_to_nue =
            Eigen::Quaterniond(state_vector[q_w_index_],
                               state_vector[q_x_index_],
                               state_vector[q_y_index_],
                               state_vector[q_z_index_]);
        state.attitude_body_to_nue.normalize();
        state.angular_rate_body_radps =
            gnc::math::Vector3(state_vector[roll_rate_index_],
                               state_vector[pitch_rate_index_],
                               state_vector[yaw_rate_index_]);
        return state;
    }

    void packState(const State& state, Eigen::VectorXd& state_vector) const {
        state_vector = Eigen::VectorXd::Zero(layout_.dimension());
        state_vector[longitude_index_] = state.longitude_rad;
        state_vector[latitude_index_] = state.latitude_rad;
        state_vector[altitude_index_] = state.altitude_m;
        state_vector[velocity_north_index_] = state.velocity_nue_mps.x();
        state_vector[velocity_up_index_] = state.velocity_nue_mps.y();
        state_vector[velocity_east_index_] = state.velocity_nue_mps.z();
        state_vector[q_w_index_] = state.attitude_body_to_nue.w();
        state_vector[q_x_index_] = state.attitude_body_to_nue.x();
        state_vector[q_y_index_] = state.attitude_body_to_nue.y();
        state_vector[q_z_index_] = state.attitude_body_to_nue.z();
        state_vector[roll_rate_index_] = state.angular_rate_body_radps.x();
        state_vector[pitch_rate_index_] = state.angular_rate_body_radps.y();
        state_vector[yaw_rate_index_] = state.angular_rate_body_radps.z();
    }

    Truth sampleTruth(const Eigen::VectorXd& state_vector, double time) const {
        const State state = unpackState(state_vector);
        Truth truth = buildTruthBasis(state, time);
        const Input input = resolveInput(truth, time);
        truth.local_acceleration_nue_mps2 = input.local_acceleration_nue_mps2;
        truth.angular_acceleration_body_radps2 =
            input.angular_acceleration_body_radps2;
        return truth;
    }

    Truth buildTruthBasis(const State& state, double time) const {
        Truth truth;
        truth.state = state;
        truth.earth_radius_m = currentReferenceRadius();
        truth.earth_rotation_rate_rad_per_s =
            earth_ ? earth_->getRotationRate() : 0.0;
        truth.sample_time_s = time;
        truth.position_ecef_m =
            earth_ ? earth_->geodeticToEcef(state.latitude_rad,
                                            state.longitude_rad,
                                            state.altitude_m)
                   : gnc::math::Vector3::Zero();
        truth.velocity_ecef_mps =
            localNueToEcefRotation(state.latitude_rad, state.longitude_rad) *
            state.velocity_nue_mps;
        return truth;
    }

    Input resolveInput(const Truth& truth, double time) const {
        if (!input_provider_) {
            return {};
        }
        return input_provider_->computeLocalSpherical6DoFInput(truth, time);
    }

    static void normalizeStateQuaternion(Eigen::VectorXd& state_vector) {
        Eigen::Quaterniond q(state_vector[6],
                             state_vector[7],
                             state_vector[8],
                             state_vector[9]);
        if (q.norm() == 0.0) {
            q = Eigen::Quaterniond::Identity();
        } else {
            q.normalize();
        }
        state_vector[6] = q.w();
        state_vector[7] = q.x();
        state_vector[8] = q.y();
        state_vector[9] = q.z();
    }

    static gnc::math::Matrix3 localNueToEcefRotation(double latitude_rad,
                                                      double longitude_rad) {
        gnc::math::Matrix3 rotation;
        rotation.col(0) = gnc::math::Vector3(-std::sin(latitude_rad) *
                                                 std::cos(longitude_rad),
                                             -std::sin(latitude_rad) *
                                                 std::sin(longitude_rad),
                                             std::cos(latitude_rad));
        rotation.col(1) = gnc::math::Vector3(std::cos(latitude_rad) *
                                                 std::cos(longitude_rad),
                                             std::cos(latitude_rad) *
                                                 std::sin(longitude_rad),
                                             std::sin(latitude_rad));
        rotation.col(2) = gnc::math::Vector3(-std::sin(longitude_rad),
                                             std::cos(longitude_rad),
                                             0.0);
        return rotation;
    }

    double currentReferenceRadius() const {
        return earth_ ? earth_->getEquatorialRadius() : reference_radius_m_;
    }

    gnc::core::StateLayout layout_;
    Eigen::VectorXd state_vector_;
    Eigen::VectorXd initial_state_vector_;
    gnc::environment::IEarth* earth_ = nullptr;
    IInputProvider* input_provider_ = nullptr;
    std::string earth_lookup_name_ = "env.earth";
    std::string input_lookup_name_ = "interaction";
    double reference_radius_m_ = 6371000.0;
    Truth truth_{};
    int longitude_index_ = -1;
    int latitude_index_ = -1;
    int altitude_index_ = -1;
    int velocity_north_index_ = -1;
    int velocity_up_index_ = -1;
    int velocity_east_index_ = -1;
    int q_w_index_ = -1;
    int q_x_index_ = -1;
    int q_y_index_ = -1;
    int q_z_index_ = -1;
    int roll_rate_index_ = -1;
    int pitch_rate_index_ = -1;
    int yaw_rate_index_ = -1;
};

} // namespace gnc::forms::local_spherical_6dof
