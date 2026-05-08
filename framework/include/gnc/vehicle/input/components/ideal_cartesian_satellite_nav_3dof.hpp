#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/forms/cartesian_3dof/interfaces/i_truth_view.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/input/interfaces/i_satellite_nav_3dof.hpp"

#include <algorithm>
#include <cmath>

namespace gnc::vehicle::input {

class IdealCartesianSatelliteNav3Dof final
    : public gnc::core::ComponentBase,
      public ISatelliteNav3Dof,
      public gnc::interfaces::IObservable {
public:
    IdealCartesianSatelliteNav3Dof()
        : ComponentBase("IdealCartesianSatelliteNav3Dof") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        truth_lookup_name_ =
            reader.optionalString("truth_lookup_name", truth_lookup_name_);
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(gnc::core::bind(truth_, truth_lookup_name_));
    }

    void initialize() override { update(0.0); }

    void update(double) override {
        const auto& truth = truth_->getCartesian3DoFTruth();
        const auto& velocity = truth.state.velocity_mps;
        const double speed = velocity.norm();
        measurement_.position_ecef_m = truth.state.position_m;
        measurement_.velocity_ecef_mps = truth.state.velocity_mps;
        measurement_.longitude_rad = 0.0;
        measurement_.latitude_rad = 0.0;
        measurement_.altitude_m = truth.state.position_m.z();
        measurement_.speed_mps = speed;
        measurement_.flight_path_angle_rad = flightPathAngle(velocity, speed);
        measurement_.heading_angle_rad = headingAngle(velocity);
    }

    const SatelliteNavMeasurement3Dof& satelliteNavMeasurement3Dof()
        const override {
        return measurement_;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addVector3("position_ecef", [this]() -> const gnc::math::Vector3& {
            return measurement_.position_ecef_m;
        });
        builder.addVector3("velocity_ecef", [this]() -> const gnc::math::Vector3& {
            return measurement_.velocity_ecef_mps;
        });
        builder.addScalar("longitude_rad",
                          [this]() { return measurement_.longitude_rad; });
        builder.addScalar("latitude_rad",
                          [this]() { return measurement_.latitude_rad; });
        builder.addScalar("altitude_m",
                          [this]() { return measurement_.altitude_m; });
        builder.addScalar("speed_mps", [this]() { return measurement_.speed_mps; });
        builder.addScalar("flight_path_angle_rad",
                          [this]() { return measurement_.flight_path_angle_rad; });
        builder.addScalar("heading_angle_rad",
                          [this]() { return measurement_.heading_angle_rad; });
        return builder.build();
    }

private:
    static double flightPathAngle(const gnc::math::Vector3& velocity,
                                  double speed) {
        if (speed <= 0.0) {
            return 0.0;
        }
        const double ratio = std::max(-1.0, std::min(1.0, velocity.z() / speed));
        return std::asin(ratio);
    }

    static double headingAngle(const gnc::math::Vector3& velocity) {
        if (velocity.x() == 0.0 && velocity.y() == 0.0) {
            return 0.0;
        }
        return std::atan2(velocity.y(), velocity.x());
    }

    gnc::forms::cartesian_3dof::ITruthView* truth_ = nullptr;
    std::string truth_lookup_name_ = "dynamics";
    SatelliteNavMeasurement3Dof measurement_{};
};

} // namespace gnc::vehicle::input
