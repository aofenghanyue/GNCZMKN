#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/environment/interfaces/i_atmosphere.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_truth_view.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/input/interfaces/i_air_data_3dof.hpp"

#include <algorithm>

namespace gnc::vehicle::input {

class IdealAirData3Dof final : public gnc::core::ComponentBase,
                               public IAirData3Dof,
                               public gnc::interfaces::IObservable {
public:
    IdealAirData3Dof() : ComponentBase("IdealAirData3Dof") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        truth_lookup_name_ =
            reader.optionalString("truth_lookup_name", truth_lookup_name_);
        atmosphere_lookup_name_ =
            reader.optionalString("atmosphere_lookup_name",
                                  atmosphere_lookup_name_);
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(
            gnc::core::bind(truth_, truth_lookup_name_),
            gnc::core::bind(atmosphere_, atmosphere_lookup_name_));
    }

    void initialize() override { update(0.0); }

    void update(double) override {
        const auto& truth = truth_->getLocalSpherical3DoFTruth();
        const auto sample = atmosphere_->sample(truth.state.altitude_m);
        measurement_.altitude_m = truth.state.altitude_m;
        measurement_.speed_mps = truth.state.speed_mps;
        measurement_.density_kg_per_m3 = sample.density_kg_per_m3;
        measurement_.pressure_pa = sample.pressure_pa;
        measurement_.temperature_k = sample.temperature_k;
        measurement_.speed_of_sound_mps = sample.speed_of_sound_mps;
        measurement_.mach_number =
            measurement_.speed_mps / std::max(1.0, sample.speed_of_sound_mps);
        measurement_.dynamic_pressure_pa =
            0.5 * measurement_.density_kg_per_m3 * measurement_.speed_mps *
            measurement_.speed_mps;
    }

    const AirDataMeasurement3Dof& airDataMeasurement3Dof() const override {
        return measurement_;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("altitude_m", [this]() { return measurement_.altitude_m; });
        builder.addScalar("speed_mps", [this]() { return measurement_.speed_mps; });
        builder.addScalar("density_kg_per_m3",
                          [this]() { return measurement_.density_kg_per_m3; });
        builder.addScalar("pressure_pa", [this]() { return measurement_.pressure_pa; });
        builder.addScalar("temperature_k",
                          [this]() { return measurement_.temperature_k; });
        builder.addScalar("speed_of_sound_mps",
                          [this]() { return measurement_.speed_of_sound_mps; });
        builder.addScalar("mach_number",
                          [this]() { return measurement_.mach_number; });
        builder.addScalar("dynamic_pressure_pa",
                          [this]() { return measurement_.dynamic_pressure_pa; });
        return builder.build();
    }

private:
    gnc::forms::local_spherical_3dof::ITruthView* truth_ = nullptr;
    gnc::environment::IAtmosphere* atmosphere_ = nullptr;
    std::string truth_lookup_name_ = "dynamics";
    std::string atmosphere_lookup_name_ = "env.atmosphere";
    AirDataMeasurement3Dof measurement_{};
};

} // namespace gnc::vehicle::input
