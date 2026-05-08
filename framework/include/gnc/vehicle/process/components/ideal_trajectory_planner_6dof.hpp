#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/input/interfaces/i_satellite_nav_6dof.hpp"
#include "gnc/vehicle/process/interfaces/i_trajectory_planner_6dof.hpp"

namespace gnc::vehicle::process {

class IdealTrajectoryPlanner6Dof final : public gnc::core::ComponentBase,
                                         public ITrajectoryPlanner6Dof,
                                         public gnc::interfaces::IObservable {
public:
    IdealTrajectoryPlanner6Dof()
        : ComponentBase("IdealTrajectoryPlanner6Dof") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        satnav_lookup_name_ =
            reader.optionalString("satnav_lookup_name", satnav_lookup_name_);
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(gnc::core::bindIfPresent(satnav_, satnav_lookup_name_));
    }

    void initialize() override { update(0.0); }

    void update(double) override {
        if (!satnav_) {
            plan_ = {};
            return;
        }
        const auto& nav = satnav_->satelliteNavMeasurement6Dof();
        plan_.desired_position_ecef_m = nav.position_ecef_m;
        plan_.desired_velocity_ecef_mps = nav.velocity_ecef_mps;
    }

    const TrajectoryPlan6Dof& trajectoryPlan6Dof() const override {
        return plan_;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addVector3("desired_position_ecef",
                           [this]() -> const gnc::math::Vector3& {
                               return plan_.desired_position_ecef_m;
                           });
        builder.addVector3("desired_velocity_ecef",
                           [this]() -> const gnc::math::Vector3& {
                               return plan_.desired_velocity_ecef_mps;
                           });
        return builder.build();
    }

private:
    gnc::vehicle::input::ISatelliteNav6Dof* satnav_ = nullptr;
    std::string satnav_lookup_name_ = "satnav";
    TrajectoryPlan6Dof plan_{};
};

} // namespace gnc::vehicle::process
