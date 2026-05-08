#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/input/interfaces/i_air_data_3dof.hpp"
#include "gnc/vehicle/input/interfaces/i_satellite_nav_3dof.hpp"
#include "gnc/vehicle/process/interfaces/i_navigation_3dof.hpp"

namespace gnc::vehicle::process {

class IdealNavigation3Dof final : public gnc::core::ComponentBase,
                                  public INavigation3Dof,
                                  public gnc::interfaces::IObservable {
public:
    IdealNavigation3Dof() : ComponentBase("IdealNavigation3Dof") {}

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
        const auto& nav = satnav_->satelliteNavMeasurement3Dof();
        state_.position_ecef_m = nav.position_ecef_m;
        state_.velocity_ecef_mps = nav.velocity_ecef_mps;
        state_.longitude_rad = nav.longitude_rad;
        state_.latitude_rad = nav.latitude_rad;
        state_.altitude_m = nav.altitude_m;
        state_.speed_mps = nav.speed_mps;
        state_.flight_path_angle_rad = nav.flight_path_angle_rad;
        state_.heading_angle_rad = nav.heading_angle_rad;
        state_.mach_number =
            air_data_ ? air_data_->airDataMeasurement3Dof().mach_number : 0.0;
    }

    const NavigationState3Dof& navigationState3Dof() const override {
        return state_;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addVector3("position_ecef", [this]() -> const gnc::math::Vector3& {
            return state_.position_ecef_m;
        });
        builder.addVector3("velocity_ecef", [this]() -> const gnc::math::Vector3& {
            return state_.velocity_ecef_mps;
        });
        builder.addScalar("altitude_m", [this]() { return state_.altitude_m; });
        builder.addScalar("speed_mps", [this]() { return state_.speed_mps; });
        builder.addScalar("mach_number", [this]() { return state_.mach_number; });
        return builder.build();
    }

private:
    gnc::vehicle::input::ISatelliteNav3Dof* satnav_ = nullptr;
    gnc::vehicle::input::IAirData3Dof* air_data_ = nullptr;
    std::string satnav_lookup_name_ = "satnav";
    std::string air_data_lookup_name_ = "air_data";
    NavigationState3Dof state_{};
};

} // namespace gnc::vehicle::process
