#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/input/interfaces/i_satellite_nav_3dof.hpp"
#include "gnc/vehicle/process/interfaces/i_trajectory_planner_3dof.hpp"

namespace gnc::vehicle::process {

class IdealTrajectoryPlanner3Dof final
    : public gnc::core::ComponentBase,
      public ITrajectoryPlanner3Dof,
      public gnc::interfaces::IObservable {
public:
    IdealTrajectoryPlanner3Dof()
        : ComponentBase("IdealTrajectoryPlanner3Dof") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        satnav_lookup_name_ =
            reader.optionalString("satnav_lookup_name", satnav_lookup_name_);
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(gnc::core::bind(satnav_, satnav_lookup_name_));
    }

    void initialize() override { update(0.0); }

    void update(double) override {
        const auto& nav = satnav_->satelliteNavMeasurement3Dof();
        command_.desired_position_ecef_m = nav.position_ecef_m;
        command_.desired_altitude_m = nav.altitude_m;
        command_.desired_speed_mps = nav.speed_mps;
    }

    const TrajectoryCommand3Dof& trajectoryCommand3Dof() const override {
        return command_;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addVector3("desired_position_ecef",
                           [this]() -> const gnc::math::Vector3& {
                               return command_.desired_position_ecef_m;
                           });
        builder.addScalar("desired_altitude_m",
                          [this]() { return command_.desired_altitude_m; });
        builder.addScalar("desired_speed_mps",
                          [this]() { return command_.desired_speed_mps; });
        return builder.build();
    }

private:
    gnc::vehicle::input::ISatelliteNav3Dof* satnav_ = nullptr;
    std::string satnav_lookup_name_ = "satnav";
    TrajectoryCommand3Dof command_{};
};

} // namespace gnc::vehicle::process
