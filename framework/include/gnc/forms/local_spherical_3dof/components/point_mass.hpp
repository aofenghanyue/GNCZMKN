#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/state_layout.hpp"
#include "gnc/environment/interfaces/i_earth.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_input_provider.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_truth_view.hpp"
#include "gnc/forms/local_spherical_3dof/internal/math.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_continuous_system.hpp"
#include "gnc/interfaces/i_observable.hpp"

#include <string>

namespace gnc::forms::local_spherical_3dof {

class PointMass final : public gnc::core::ComponentBase,
                        public gnc::interfaces::IContinuousSystem,
                        public ITruthView,
                        public gnc::interfaces::IObservable {
public:
    PointMass() : ComponentBase("LocalSpherical3DoFPointMass") {
        longitude_index_ = layout_.addVariable("longitude_rad");
        latitude_index_ = layout_.addVariable("latitude_rad");
        altitude_index_ = layout_.addVariable("altitude_m");
        speed_index_ = layout_.addVariable("speed_mps");
        flight_path_index_ = layout_.addVariable("flight_path_angle_rad");
        heading_index_ = layout_.addVariable("heading_angle_rad");
        state_vector_ = Eigen::VectorXd::Zero(layout_.dimension());
        initial_state_vector_ = state_vector_;
    }

    void configure(const gnc::core::ConfigNode& config) override {
        configure(config, "config");
    }

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        const auto initial_state = reader.requiredObject("initial_state");
        initial_state_vector_[longitude_index_] =
            initial_state.requiredDouble("longitude_rad");
        initial_state_vector_[latitude_index_] =
            initial_state.requiredDouble("latitude_rad");
        initial_state_vector_[altitude_index_] =
            initial_state.requiredDouble("altitude_m");
        initial_state_vector_[speed_index_] =
            initial_state.requiredDouble("speed_mps");
        initial_state_vector_[flight_path_index_] =
            initial_state.requiredDouble("flight_path_angle_rad");
        initial_state_vector_[heading_index_] =
            initial_state.requiredDouble("heading_angle_rad");
        initial_state.validateNoUnknownKeys();

        earth_lookup_name_ = reader.optionalString("earth_lookup_name",
                                                   earth_lookup_name_);
        input_lookup_name_ = reader.optionalString("input_lookup_name",
                                                   input_lookup_name_);
        launch_azimuth_rad_ = reader.requiredDouble("launch_azimuth_rad");
        reference_radius_m_ = reader.optionalDouble("reference_radius_m",
                                                   reference_radius_m_);
        reader.validateNoUnknownKeys();
        state_vector_ = initial_state_vector_;
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(gnc::core::bind(earth_, earth_lookup_name_));
        input_provider_ = registry.requireByName<IInputProvider>(input_lookup_name_);
    }

    void initialize() override { publish(getSimTime()); }

    void publish(double time) override { refreshTruth(time); }

    void update(double) override {}

    const gnc::core::StateLayout& getStateLayout() const override { return layout_; }

    void computeDerivatives(double time,
                            const Eigen::VectorXd& state_vector,
                            Eigen::VectorXd& derivative_vector) const override {
        const State state = unpackState(state_vector);
        const Truth truth_basis = buildTruthBasis(state, time);
        const Input input = resolveInput(truth_basis, time);
        const auto derivative = internal::computeStateDerivatives(
            state,
            input.local_acceleration_nue_mps2,
            truth_basis.earth_radius_m,
            truth_basis.earth_rotation_rate_rad_per_s);

        derivative_vector = Eigen::VectorXd::Zero(layout_.dimension());
        derivative_vector[longitude_index_] = derivative.longitude_rate_rad_per_s;
        derivative_vector[latitude_index_] = derivative.latitude_rate_rad_per_s;
        derivative_vector[altitude_index_] = derivative.altitude_rate_m_per_s;
        derivative_vector[speed_index_] = derivative.speed_rate_m_per_s2;
        derivative_vector[flight_path_index_] =
            derivative.flight_path_angle_rate_rad_per_s;
        derivative_vector[heading_index_] = derivative.heading_angle_rate_rad_per_s;
    }

    const Eigen::VectorXd& getState() const override { return state_vector_; }
    void setState(const Eigen::VectorXd& state) override { state_vector_ = state; }
    Eigen::VectorXd getInitialState() const override { return initial_state_vector_; }

    gnc::math::Vector3 getPosition() const {
        return sampleTruth(getSimTime()).position_ecef_m;
    }

    gnc::math::Vector3 getVelocity() const {
        return sampleTruth(getSimTime()).velocity_ecef_mps;
    }

    double getSpeed() const { return state_vector_[speed_index_]; }
    double getAltitude() const { return state_vector_[altitude_index_]; }

    gnc::math::Vector3 getVelocityInLaunchFrame() const {
        return sampleTruth(getSimTime()).velocity_launch_mps;
    }

    const Truth& getLocalSpherical3DoFTruth() const override { return truth_; }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("longitude_rad", [this]() { return truth_.state.longitude_rad; });
        builder.addScalar("latitude_rad", [this]() { return truth_.state.latitude_rad; });
        builder.addScalar("altitude_m", [this]() { return truth_.state.altitude_m; });
        builder.addScalar("speed_mps", [this]() { return truth_.state.speed_mps; });
        builder.addScalar("flight_path_angle_rad",
                          [this]() { return truth_.state.flight_path_angle_rad; });
        builder.addScalar("heading_angle_rad",
                          [this]() { return truth_.state.heading_angle_rad; });
        builder.addVector3("position_ecef", [this]() -> const gnc::math::Vector3& {
            return truth_.position_ecef_m;
        });
        builder.addVector3("velocity_ecef", [this]() -> const gnc::math::Vector3& {
            return truth_.velocity_ecef_mps;
        });
        builder.addVector3("local_velocity_nue", [this]() -> const gnc::math::Vector3& {
            return truth_.local_velocity_nue_mps;
        });
        builder.addVector3("local_acceleration_nue",
                           [this]() -> const gnc::math::Vector3& {
                               return truth_.local_acceleration_nue_mps2;
                           });
        builder.addVector3("velocity_launch", [this]() -> const gnc::math::Vector3& {
            return truth_.velocity_launch_mps;
        });
        return builder.build();
    }

private:
    State unpackState(const Eigen::VectorXd& state_vector) const {
        State state;
        state.longitude_rad = state_vector[longitude_index_];
        state.latitude_rad = state_vector[latitude_index_];
        state.altitude_m = state_vector[altitude_index_];
        state.speed_mps = state_vector[speed_index_];
        state.flight_path_angle_rad = state_vector[flight_path_index_];
        state.heading_angle_rad = state_vector[heading_index_];
        return state;
    }

    Truth buildTruthBasis(const State& state, double sample_time) const {
        Truth truth;
        truth.state = state;
        truth.launch_azimuth_rad = launch_azimuth_rad_;
        truth.earth_radius_m = currentReferenceRadius();
        truth.earth_rotation_rate_rad_per_s = earth_ ? earth_->getRotationRate() : 0.0;
        truth.sample_time_s = sample_time;
        truth.position_ecef_m = earth_ ? earth_->geodeticToEcef(state.latitude_rad,
                                                                state.longitude_rad,
                                                                state.altitude_m)
                                       : gnc::math::Vector3::Zero();
        truth.local_velocity_nue_mps = internal::localVelocityNue(state);
        truth.drag_axis_nue = internal::velocityBasisK1(state);
        truth.lift_up_axis_nue = internal::velocityBasisK2(state);
        truth.lift_side_axis_nue = internal::velocityBasisK3(state);
        truth.velocity_ecef_mps =
            internal::localNueToEcefRotation(state.latitude_rad, state.longitude_rad) *
            truth.local_velocity_nue_mps;
        truth.velocity_launch_mps = internal::velocityLaunchFrame(truth);
        return truth;
    }

    Input resolveInput(const Truth& truth, double time) const {
        if (!input_provider_) {
            return {};
        }
        return input_provider_->computeLocalSpherical3DoFInput(truth, time);
    }

    Truth sampleTruth(double sample_time) const {
        const State state = unpackState(state_vector_);
        Truth truth = buildTruthBasis(state, sample_time);
        truth.local_acceleration_nue_mps2 =
            resolveInput(truth, sample_time).local_acceleration_nue_mps2;
        return truth;
    }

    void refreshTruth(double sample_time) {
        truth_ = sampleTruth(sample_time);
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
    double launch_azimuth_rad_ = 0.0;
    double reference_radius_m_ = 6371000.0;
    int longitude_index_ = -1;
    int latitude_index_ = -1;
    int altitude_index_ = -1;
    int speed_index_ = -1;
    int flight_path_index_ = -1;
    int heading_index_ = -1;
    Truth truth_{};
};

} // namespace gnc::forms::local_spherical_3dof
