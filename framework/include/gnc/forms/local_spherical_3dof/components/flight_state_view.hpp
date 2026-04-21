#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/environment/interfaces/i_atmosphere.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_flight_state_view.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_truth_view.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/process/interfaces/i_aero_guidance_provider.hpp"

#include <algorithm>

namespace gnc::forms::local_spherical_3dof {

class FlightStateView final : public gnc::core::ComponentBase,
                              public IFlightStateView,
                              public gnc::interfaces::IObservable {
public:
    FlightStateView() : ComponentBase("LocalSpherical3DoFFlightStateView") {}

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(
            gnc::core::bind(truth_view_, "dynamics"),
            gnc::core::bind(atmosphere_, "env.atmosphere"),
            gnc::core::bindIfPresent(command_provider_, "guidance"));
    }

    void initialize() override { refreshSample(); }

    void update(double) override { refreshSample(); }

    const FlightState& getFlightState() const override {
        return sample_;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("longitude_rad", [this]() { return sample_.longitude_rad; });
        builder.addScalar("latitude_rad", [this]() { return sample_.latitude_rad; });
        builder.addScalar("altitude_m", [this]() { return sample_.altitude_m; });
        builder.addScalar("speed_mps", [this]() { return sample_.speed_mps; });
        builder.addScalar("flight_path_angle_rad",
                          [this]() { return sample_.flight_path_angle_rad; });
        builder.addScalar("heading_angle_rad",
                          [this]() { return sample_.heading_angle_rad; });
        builder.addVector3("local_velocity_nue", [this]() -> const gnc::math::Vector3& {
            return sample_.local_velocity_nue_mps;
        });
        builder.addVector3("local_acceleration_nue",
                           [this]() -> const gnc::math::Vector3& {
                               return sample_.local_acceleration_nue_mps2;
                           });
        builder.addScalar("angle_of_attack_rad",
                          [this]() { return sample_.angle_of_attack_rad; });
        builder.addScalar("bank_angle_rad", [this]() { return sample_.bank_angle_rad; });
        builder.addScalar("dynamic_pressure_pa",
                          [this]() { return sample_.dynamic_pressure_pa; });
        builder.addScalar("density_kg_per_m3",
                          [this]() { return sample_.density_kg_per_m3; });
        builder.addScalar("pressure_pa", [this]() { return sample_.pressure_pa; });
        builder.addScalar("temperature_k", [this]() { return sample_.temperature_k; });
        builder.addScalar("speed_of_sound_mps",
                          [this]() { return sample_.speed_of_sound_mps; });
        builder.addScalar("mach_number", [this]() { return sample_.mach_number; });
        return builder.build();
    }

private:
    gnc::vehicle::process::AeroGuidanceCommand currentCommand() const {
        if (command_provider_ && command_provider_->isGuidanceActive()) {
            return command_provider_->getAeroGuidanceCommand();
        }
        return {};
    }

    void refreshSample() {
        const auto& truth = truth_view_->getLocalSpherical3DoFTruth();
        const auto atmosphere_sample = atmosphere_->sample(truth.state.altitude_m);
        const auto command = currentCommand();

        sample_.longitude_rad = truth.state.longitude_rad;
        sample_.latitude_rad = truth.state.latitude_rad;
        sample_.altitude_m = truth.state.altitude_m;
        sample_.speed_mps = truth.state.speed_mps;
        sample_.flight_path_angle_rad = truth.state.flight_path_angle_rad;
        sample_.heading_angle_rad = truth.state.heading_angle_rad;
        sample_.local_velocity_nue_mps = truth.local_velocity_nue_mps;
        sample_.local_acceleration_nue_mps2 = truth.local_acceleration_nue_mps2;
        sample_.angle_of_attack_rad = command.angle_of_attack_rad;
        sample_.bank_angle_rad = command.bank_angle_rad;
        sample_.density_kg_per_m3 = atmosphere_sample.density_kg_per_m3;
        sample_.pressure_pa = atmosphere_sample.pressure_pa;
        sample_.temperature_k = atmosphere_sample.temperature_k;
        sample_.speed_of_sound_mps = atmosphere_sample.speed_of_sound_mps;
        sample_.dynamic_pressure_pa =
            0.5 * sample_.density_kg_per_m3 * sample_.speed_mps * sample_.speed_mps;
        sample_.mach_number =
            sample_.speed_mps / std::max(1.0, sample_.speed_of_sound_mps);
    }

    ITruthView* truth_view_ = nullptr;
    gnc::environment::IAtmosphere* atmosphere_ = nullptr;
    gnc::vehicle::process::IAeroGuidanceProvider* command_provider_ = nullptr;
    FlightState sample_{};
};

} // namespace gnc::forms::local_spherical_3dof
