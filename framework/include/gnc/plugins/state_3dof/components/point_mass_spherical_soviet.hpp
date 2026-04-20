#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/core/state_layout.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_continuous_system.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/plugins/environment/interfaces/i_earth.hpp"
#include "gnc/plugins/state_3dof/interfaces/i_acceleration_provider_3dof.hpp"
#include "gnc/plugins/state_3dof/interfaces/i_soviet_spherical_state_3dof.hpp"
#include "gnc/plugins/state_3dof/interfaces/i_state_solver_3dof.hpp"
#include "gnc/plugins/state_3dof/interfaces/i_velocity_direction_provider.hpp"
#include "gnc/plugins/state_3dof/internal/soviet_spherical_3dof_math.hpp"

#include <string>

namespace gnc::plugins::state_3dof {

class PointMassSphericalSoviet final : public gnc::core::ComponentBase,
                                       public gnc::interfaces::IContinuousSystem,
                                       public IStateSolver3DOF,
                                       public ISovietSphericalState3DOF,
                                       public IVelocityDirectionProvider,
                                       public gnc::interfaces::IObservable {
public:
    PointMassSphericalSoviet() : ComponentBase("PointMassSphericalSoviet") {
        longitude_index_ = layout_.addVariable("longitude_rad");
        latitude_index_ = layout_.addVariable("latitude_rad");
        altitude_index_ = layout_.addVariable("altitude_m");
        speed_index_ = layout_.addVariable("speed_mps");
        flight_path_index_ = layout_.addVariable("flight_path_angle_rad");
        heading_index_ = layout_.addVariable("heading_angle_rad");
        state_ = Eigen::VectorXd::Zero(layout_.dimension());
        initial_state_ = state_;
    }

    void configure(const gnc::core::ConfigNode& config) override {
        const auto& initial_state = config["initial_state"];
        initial_state_[longitude_index_] = initial_state["longitude_rad"].asDouble(0.0);
        initial_state_[latitude_index_] = initial_state["latitude_rad"].asDouble(0.0);
        initial_state_[altitude_index_] = initial_state["altitude_m"].asDouble(30000.0);
        initial_state_[speed_index_] = initial_state["speed_mps"].asDouble(3500.0);
        initial_state_[flight_path_index_] =
            initial_state["flight_path_angle_rad"].asDouble(-0.05);
        initial_state_[heading_index_] =
            initial_state["heading_angle_rad"].asDouble(0.0);

        earth_lookup_name_ = config["earth_lookup_name"].asString(earth_lookup_name_);
        acceleration_lookup_name_ =
            config["acceleration_lookup_name"].asString(acceleration_lookup_name_);
        launch_azimuth_rad_ =
            config["launch_azimuth_rad"].asDouble(launch_azimuth_rad_);
        reference_radius_m_ = config["reference_radius_m"].asDouble(reference_radius_m_);
        state_ = initial_state_;
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(gnc::core::bind(earth_, earth_lookup_name_),
                         gnc::core::bind(acceleration_provider_,
                                         acceleration_lookup_name_));
    }

    const gnc::core::StateLayout& getStateLayout() const override { return layout_; }

    void computeDerivatives(double time,
                            const Eigen::VectorXd& state_vector,
                            Eigen::VectorXd& derivative_vector) const override {
        const SovietSphericalState3DOF state = unpackState(state_vector);
        const gnc::math::Vector3 local_acceleration_nue =
            acceleration_provider_->computeLocalAccelerationNue(state, time);
        const auto derivative = internal::computeStateDerivatives(
            state,
            local_acceleration_nue,
            currentReferenceRadius(),
            earth_->getRotationRate());

        derivative_vector = Eigen::VectorXd::Zero(layout_.dimension());
        derivative_vector[longitude_index_] = derivative.longitude_rate_rad_per_s;
        derivative_vector[latitude_index_] = derivative.latitude_rate_rad_per_s;
        derivative_vector[altitude_index_] = derivative.altitude_rate_m_per_s;
        derivative_vector[speed_index_] = derivative.speed_rate_m_per_s2;
        derivative_vector[flight_path_index_] =
            derivative.flight_path_angle_rate_rad_per_s;
        derivative_vector[heading_index_] = derivative.heading_angle_rate_rad_per_s;
    }

    const Eigen::VectorXd& getState() const override { return state_; }
    void setState(const Eigen::VectorXd& state) override { state_ = state; }
    Eigen::VectorXd getInitialState() const override { return initial_state_; }

    void update(double) override {}

    gnc::math::Vector3 getPosition() const override {
        const auto state = getSovietSphericalState();
        return earth_->geodeticToEcef(
            state.latitude_rad,
            state.longitude_rad,
            state.altitude_m);
    }

    gnc::math::Vector3 getVelocity() const override {
        const auto state = getSovietSphericalState();
        const gnc::math::Vector3 local_velocity =
            internal::localVelocityNue(state);
        return internal::localNueToEcefRotation(state.latitude_rad,
                                                state.longitude_rad) *
               local_velocity;
    }

    double getSpeed() const override { return state_[speed_index_]; }
    double getAltitude() const override { return state_[altitude_index_]; }

    SovietSphericalState3DOF getSovietSphericalState() const override {
        return unpackState(state_);
    }

    gnc::math::Vector3 getVelocityInLaunchFrame() const override {
        const gnc::math::Vector3 local_velocity_nue =
            internal::localVelocityNue(getSovietSphericalState());
        const double cos_azimuth = std::cos(launch_azimuth_rad_);
        const double sin_azimuth = std::sin(launch_azimuth_rad_);

        return gnc::math::Vector3(
            local_velocity_nue.x() * cos_azimuth + local_velocity_nue.z() * sin_azimuth,
            local_velocity_nue.y(),
            -local_velocity_nue.x() * sin_azimuth + local_velocity_nue.z() * cos_azimuth);
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("longitude_rad", [this]() { return state_[longitude_index_]; });
        builder.addScalar("latitude_rad", [this]() { return state_[latitude_index_]; });
        builder.addScalar("altitude_m", [this]() { return state_[altitude_index_]; });
        builder.addScalar("speed_mps", [this]() { return state_[speed_index_]; });
        builder.addScalar("flight_path_angle_rad",
                          [this]() { return state_[flight_path_index_]; });
        builder.addScalar("heading_angle_rad",
                          [this]() { return state_[heading_index_]; });
        builder.addVector3("position_ecef", [this]() -> const gnc::math::Vector3& {
            position_ecef_cache_ = getPosition();
            return position_ecef_cache_;
        });
        builder.addVector3("velocity_ecef", [this]() -> const gnc::math::Vector3& {
            velocity_ecef_cache_ = getVelocity();
            return velocity_ecef_cache_;
        });
        builder.addVector3("local_velocity_nue", [this]() -> const gnc::math::Vector3& {
            local_velocity_cache_ =
                internal::localVelocityNue(getSovietSphericalState());
            return local_velocity_cache_;
        });
        builder.addVector3("velocity_launch", [this]() -> const gnc::math::Vector3& {
            velocity_launch_cache_ = getVelocityInLaunchFrame();
            return velocity_launch_cache_;
        });
        return builder.build();
    }

private:
    SovietSphericalState3DOF unpackState(const Eigen::VectorXd& state_vector) const {
        SovietSphericalState3DOF state;
        state.longitude_rad = state_vector[longitude_index_];
        state.latitude_rad = state_vector[latitude_index_];
        state.altitude_m = state_vector[altitude_index_];
        state.speed_mps = state_vector[speed_index_];
        state.flight_path_angle_rad = state_vector[flight_path_index_];
        state.heading_angle_rad = state_vector[heading_index_];
        return state;
    }

    double currentReferenceRadius() const {
        return earth_ ? earth_->getEquatorialRadius() : reference_radius_m_;
    }

    gnc::core::StateLayout layout_;
    Eigen::VectorXd state_;
    Eigen::VectorXd initial_state_;
    gnc::plugins::environment::IEarth* earth_ = nullptr;
    IAccelerationProvider3DOF* acceleration_provider_ = nullptr;
    std::string earth_lookup_name_ = "env.earth";
    std::string acceleration_lookup_name_ = "bridge";
    double launch_azimuth_rad_ = 0.0;
    double reference_radius_m_ = 6371000.0;
    int longitude_index_ = -1;
    int latitude_index_ = -1;
    int altitude_index_ = -1;
    int speed_index_ = -1;
    int flight_path_index_ = -1;
    int heading_index_ = -1;
    mutable gnc::math::Vector3 position_ecef_cache_ = gnc::math::Vector3::Zero();
    mutable gnc::math::Vector3 velocity_ecef_cache_ = gnc::math::Vector3::Zero();
    mutable gnc::math::Vector3 local_velocity_cache_ = gnc::math::Vector3::Zero();
    mutable gnc::math::Vector3 velocity_launch_cache_ = gnc::math::Vector3::Zero();
};

} // namespace gnc::plugins::state_3dof
