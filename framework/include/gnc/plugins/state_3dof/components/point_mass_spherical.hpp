#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/core/state_layout.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_continuous_system.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/plugins/aero/interfaces/i_aero_model.hpp"
#include "gnc/plugins/environment/interfaces/i_atmosphere.hpp"
#include "gnc/plugins/environment/interfaces/i_earth.hpp"
#include "gnc/plugins/environment/interfaces/i_gravity.hpp"
#include "gnc/plugins/soviet_coord/internal/provider_contracts.hpp"
#include "gnc/plugins/state_3dof/interfaces/i_flight_command_provider_3dof.hpp"
#include "gnc/plugins/state_3dof/interfaces/i_state_solver_3dof.hpp"

#include <algorithm>
#include <cmath>

namespace gnc::plugins::state_3dof {

class PointMassSpherical final : public gnc::core::ComponentBase,
                                 public gnc::interfaces::IContinuousSystem,
                                 public IStateSolver3DOF,
                                 public gnc::plugins::soviet_coord::IVelocityDirectionProvider,
                                 public gnc::interfaces::IObservable {
public:
    PointMassSpherical() : ComponentBase("PointMassSpherical") {
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
            initial_state["heading_angle_rad"].asDouble(1.5707963267948966);

        launch_azimuth_rad_ = config["launch_azimuth_rad"].asDouble(launch_azimuth_rad_);
        mass_kg_ = config["mass_kg"].asDouble(mass_kg_);
        reference_radius_m_ = config["reference_radius_m"].asDouble(reference_radius_m_);
        state_ = initial_state_;
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(
            gnc::core::bind(atmosphere_, "atmosphere"),
            gnc::core::bind(gravity_, "gravity"),
            gnc::core::bind(aero_model_, "aero"),
            gnc::core::bind(guidance_, "guidance"),
            gnc::core::bindIfPresent(earth_, "earth"));
    }

    const gnc::core::StateLayout& getStateLayout() const override { return layout_; }

    void computeDerivatives(double,
                            const Eigen::VectorXd& state,
                            Eigen::VectorXd& derivative) const override {
        derivative = Eigen::VectorXd::Zero(layout_.dimension());

        const double latitude = state[latitude_index_];
        const double altitude = state[altitude_index_];
        const double speed = std::max(1.0, state[speed_index_]);
        const double flight_path_angle = state[flight_path_index_];
        const double heading = state[heading_index_];
        const double angle_of_attack = guidance_->isActive()
                                           ? guidance_->getFlightCommand().angle_of_attack_rad
                                           : 0.0;
        const double bank_angle = guidance_->isActive()
                                      ? guidance_->getFlightCommand().bank_angle_rad
                                      : 0.0;
        const double gravity = gravity_->getGravityMagnitude(altitude);
        const double density = atmosphere_->getDensity(altitude);
        const double speed_of_sound = atmosphere_->getSpeedOfSound(altitude);
        const double mach = speed / std::max(1.0, speed_of_sound);
        const auto coefficients =
            aero_model_->computeCoefficients(angle_of_attack, 0.0, mach);
        const double dynamic_pressure = 0.5 * density * speed * speed;
        const double lift = dynamic_pressure * aero_model_->getReferenceArea() *
                            coefficients.lift_coefficient;
        const double drag = dynamic_pressure * aero_model_->getReferenceArea() *
                            coefficients.drag_coefficient;
        const double radius = currentReferenceRadius() + altitude;
        const double cos_latitude = std::max(1e-6, std::cos(latitude));
        const double cos_flight_path = std::max(1e-6, std::cos(flight_path_angle));

        derivative[longitude_index_] =
            speed * cos_flight_path * std::sin(heading) / (radius * cos_latitude);
        derivative[latitude_index_] =
            speed * cos_flight_path * std::cos(heading) / radius;
        derivative[altitude_index_] = speed * std::sin(flight_path_angle);
        derivative[speed_index_] = -drag / mass_kg_ - gravity * std::sin(flight_path_angle);
        derivative[flight_path_index_] =
            (lift * std::cos(bank_angle)) / (mass_kg_ * speed) +
            (speed / radius - gravity / speed) * cos_flight_path;
        derivative[heading_index_] =
            (lift * std::sin(bank_angle)) / (mass_kg_ * speed * cos_flight_path) +
            speed * cos_flight_path * std::sin(heading) * std::tan(latitude) / radius;
    }

    const Eigen::VectorXd& getState() const override { return state_; }
    void setState(const Eigen::VectorXd& state) override { state_ = state; }
    Eigen::VectorXd getInitialState() const override { return initial_state_; }
    void update(double) override {}

    gnc::math::Vector3 getPosition() const override {
        const double longitude = state_[longitude_index_];
        const double latitude = state_[latitude_index_];
        const double altitude = state_[altitude_index_];
        const double radius = currentReferenceRadius() + altitude;

        return gnc::math::Vector3(
            radius * std::cos(latitude) * std::cos(longitude),
            radius * std::cos(latitude) * std::sin(longitude),
            radius * std::sin(latitude));
    }

    gnc::math::Vector3 getVelocity() const override {
        const double speed = state_[speed_index_];
        const double flight_path_angle = state_[flight_path_index_];
        const double heading = state_[heading_index_];
        const double longitude = state_[longitude_index_];
        const double latitude = state_[latitude_index_];

        const double north_speed = speed * std::cos(flight_path_angle) * std::cos(heading);
        const double east_speed = speed * std::cos(flight_path_angle) * std::sin(heading);
        const double up_speed = speed * std::sin(flight_path_angle);

        const gnc::math::Vector3 north_axis(
            -std::sin(latitude) * std::cos(longitude),
            -std::sin(latitude) * std::sin(longitude),
            std::cos(latitude));
        const gnc::math::Vector3 east_axis(-std::sin(longitude), std::cos(longitude), 0.0);
        const gnc::math::Vector3 up_axis(
            std::cos(latitude) * std::cos(longitude),
            std::cos(latitude) * std::sin(longitude),
            std::sin(latitude));

        return north_speed * north_axis + east_speed * east_axis + up_speed * up_axis;
    }

    double getMass() const override { return mass_kg_; }
    double getSpeed() const override { return state_[speed_index_]; }
    double getAltitude() const override { return state_[altitude_index_]; }

    gnc::math::Vector3 getVelocityInLaunchFrame() const override {
        const double speed = state_[speed_index_];
        const double flight_path_angle = state_[flight_path_index_];
        const double heading = state_[heading_index_];
        const double heading_offset = heading - launch_azimuth_rad_;
        const double horizontal_speed = speed * std::cos(flight_path_angle);

        return gnc::math::Vector3(horizontal_speed * std::cos(heading_offset),
                                  speed * std::sin(flight_path_angle),
                                  horizontal_speed * std::sin(heading_offset));
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("longitude_rad", [this]() { return state_[longitude_index_]; });
        builder.addScalar("latitude_rad", [this]() { return state_[latitude_index_]; });
        builder.addScalar("altitude_m", [this]() { return state_[altitude_index_]; });
        builder.addScalar("speed_mps", [this]() { return state_[speed_index_]; });
        builder.addScalar("flight_path_angle_rad",
                          [this]() { return state_[flight_path_index_]; });
        builder.addScalar("heading_angle_rad", [this]() { return state_[heading_index_]; });
        builder.addVector3("velocity_launch", [this]() -> const gnc::math::Vector3& {
            velocity_launch_cache_ = getVelocityInLaunchFrame();
            return velocity_launch_cache_;
        });
        return builder.build();
    }

private:
    double currentReferenceRadius() const {
        return earth_ ? earth_->getEquatorialRadius() : reference_radius_m_;
    }

    gnc::core::StateLayout layout_;
    Eigen::VectorXd state_;
    Eigen::VectorXd initial_state_;
    gnc::plugins::environment::IAtmosphere* atmosphere_ = nullptr;
    gnc::plugins::environment::IGravity* gravity_ = nullptr;
    gnc::plugins::environment::IEarth* earth_ = nullptr;
    gnc::plugins::aero::IAeroModel* aero_model_ = nullptr;
    gnc::plugins::state_3dof::IFlightCommandProvider3DOF* guidance_ = nullptr;
    double mass_kg_ = 900.0;
    double launch_azimuth_rad_ = 1.5707963267948966;
    double reference_radius_m_ = 6371000.0;
    int longitude_index_ = -1;
    int latitude_index_ = -1;
    int altitude_index_ = -1;
    int speed_index_ = -1;
    int flight_path_index_ = -1;
    int heading_index_ = -1;
    mutable gnc::math::Vector3 velocity_launch_cache_ = gnc::math::Vector3::Zero();
};

} // namespace gnc::plugins::state_3dof
