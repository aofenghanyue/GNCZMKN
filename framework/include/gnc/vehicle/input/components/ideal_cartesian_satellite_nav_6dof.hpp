#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/forms/cartesian_6dof/interfaces/i_truth_view.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/input/interfaces/i_satellite_nav_6dof.hpp"

namespace gnc::vehicle::input {

class IdealCartesianSatelliteNav6Dof final
    : public gnc::core::ComponentBase,
      public ISatelliteNav6Dof,
      public gnc::interfaces::IObservable {
public:
    IdealCartesianSatelliteNav6Dof()
        : ComponentBase("IdealCartesianSatelliteNav6Dof") {}

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
        const auto& truth = truth_->getCartesian6DoFTruth();
        measurement_.position_ecef_m = truth.state.position_m;
        measurement_.velocity_ecef_mps = truth.state.velocity_mps;
        measurement_.attitude_body_to_nue =
            truth.state.attitude_body_to_frame;
        measurement_.longitude_rad = 0.0;
        measurement_.latitude_rad = 0.0;
        measurement_.altitude_m = truth.state.position_m.z();
    }

    const SatelliteNavMeasurement6Dof& satelliteNavMeasurement6Dof()
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
        builder.addQuaternion("attitude_body_to_nue",
                              [this]() -> const Eigen::Quaterniond& {
                                  return measurement_.attitude_body_to_nue;
                              });
        builder.addScalar("longitude_rad",
                          [this]() { return measurement_.longitude_rad; });
        builder.addScalar("latitude_rad",
                          [this]() { return measurement_.latitude_rad; });
        builder.addScalar("altitude_m",
                          [this]() { return measurement_.altitude_m; });
        return builder.build();
    }

private:
    gnc::forms::cartesian_6dof::ITruthView* truth_ = nullptr;
    std::string truth_lookup_name_ = "dynamics";
    SatelliteNavMeasurement6Dof measurement_{};
};

} // namespace gnc::vehicle::input
