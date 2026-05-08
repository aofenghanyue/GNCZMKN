#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/input/interfaces/i_air_data_6dof.hpp"
#include "gnc/vehicle/input/interfaces/i_satellite_nav_6dof.hpp"
#include "gnc/vehicle/process/interfaces/i_navigation_6dof.hpp"

namespace gnc::vehicle::process {

class IdealNavigation6Dof final : public gnc::core::ComponentBase,
                                  public INavigation6Dof,
                                  public gnc::interfaces::IObservable {
public:
    IdealNavigation6Dof() : ComponentBase("IdealNavigation6Dof") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        satnav_lookup_name_ =
            reader.optionalString("satnav_lookup_name", satnav_lookup_name_);
        air_data_lookup_name_ =
            reader.optionalString("air_data_lookup_name", air_data_lookup_name_);
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(
            gnc::core::bind(satnav_, satnav_lookup_name_),
            gnc::core::bindIfPresent(air_data_, air_data_lookup_name_));
    }

    void initialize() override { update(0.0); }

    void update(double) override {
        const auto& nav = satnav_->satelliteNavMeasurement6Dof();
        solution_.position_ecef_m = nav.position_ecef_m;
        solution_.velocity_ecef_mps = nav.velocity_ecef_mps;
        solution_.altitude_m = nav.altitude_m;
        solution_.mach_number =
            air_data_ ? air_data_->airDataMeasurement6Dof().mach_number : 0.0;
    }

    const NavigationSolution6Dof& navigationSolution6Dof() const override {
        return solution_;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addVector3("position_ecef", [this]() -> const gnc::math::Vector3& {
            return solution_.position_ecef_m;
        });
        builder.addVector3("velocity_ecef", [this]() -> const gnc::math::Vector3& {
            return solution_.velocity_ecef_mps;
        });
        builder.addScalar("altitude_m", [this]() { return solution_.altitude_m; });
        builder.addScalar("mach_number", [this]() { return solution_.mach_number; });
        return builder.build();
    }

private:
    gnc::vehicle::input::ISatelliteNav6Dof* satnav_ = nullptr;
    gnc::vehicle::input::IAirData6Dof* air_data_ = nullptr;
    std::string satnav_lookup_name_ = "satnav";
    std::string air_data_lookup_name_ = "air_data";
    NavigationSolution6Dof solution_{};
};

} // namespace gnc::vehicle::process
