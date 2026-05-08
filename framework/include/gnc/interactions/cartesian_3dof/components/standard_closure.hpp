#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/forms/cartesian_3dof/interfaces/i_input_provider.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/input/interfaces/i_air_data_3dof.hpp"
#include "gnc/vehicle/output/interfaces/i_actuator_3dof.hpp"
#include "gnc/vehicle/output/interfaces/i_aero_model.hpp"
#include "gnc/vehicle/output/interfaces/i_constant_mass.hpp"
#include "gnc/vehicle/output/interfaces/i_continuous_mass.hpp"
#include "gnc/vehicle/output/interfaces/i_force_provider.hpp"
#include "gnc/vehicle/process/interfaces/i_guidance_3dof.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace gnc::interactions::cartesian_3dof {

class StandardClosure final
    : public gnc::core::ComponentBase,
      public gnc::forms::cartesian_3dof::IInputProvider,
      public gnc::interfaces::IObservable {
public:
    StandardClosure() : ComponentBase("Cartesian3DoFStandardClosure") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        guidance_lookup_name_ =
            reader.optionalString("guidance_lookup_name", guidance_lookup_name_);
        air_data_lookup_name_ =
            reader.optionalString("air_data_lookup_name", air_data_lookup_name_);
        aero_lookup_name_ =
            reader.optionalString("aero_lookup_name", aero_lookup_name_);
        propulsion_lookup_name_ =
            reader.optionalString("propulsion_lookup_name",
                                  propulsion_lookup_name_);
        actuator_lookup_name_ =
            reader.optionalString("actuator_lookup_name", actuator_lookup_name_);
        mass_lookup_name_ =
            reader.optionalString("mass_lookup_name", mass_lookup_name_);

        const auto acceleration =
            reader.optionalDoubleArray("acceleration_mps2", {0.0, 0.0, 0.0}, 3);
        configured_input_.acceleration_mps2 =
            gnc::math::Vector3(acceleration[0], acceleration[1], acceleration[2]);
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(
            gnc::core::bindIfPresent(guidance_, guidance_lookup_name_),
            gnc::core::bindIfPresent(air_data_, air_data_lookup_name_),
            gnc::core::bind(aero_, aero_lookup_name_),
            gnc::core::bind(propulsion_, propulsion_lookup_name_),
            gnc::core::bindIfPresent(actuator_, actuator_lookup_name_),
            gnc::core::bindIfPresent(constant_mass_, mass_lookup_name_),
            gnc::core::bindIfPresent(continuous_mass_, mass_lookup_name_));

        if ((constant_mass_ != nullptr) == (continuous_mass_ != nullptr)) {
            throw std::runtime_error(
                "interaction.cartesian_3dof.standard requires exactly one mass "
                "provider named 'mass' implementing either IConstantMass or "
                "IContinuousMass.");
        }
    }

    void initialize() override { update(0.0); }

    void update(double) override { refreshDiagnostics(nullptr); }

    gnc::forms::cartesian_3dof::Input computeCartesian3DoFInput(
        const gnc::forms::cartesian_3dof::Truth& truth,
        double) const override {
        refreshDiagnostics(&truth);
        return currentInput();
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addVector3("acceleration",
                           [this]() -> const gnc::math::Vector3& {
                               return last_input_.acceleration_mps2;
                           });
        builder.addVector3("propulsion_force_n",
                           [this]() -> const gnc::math::Vector3& {
                               return propulsion_force_n_;
                           });
        builder.addScalar("mass_kg", [this]() { return mass_kg_; });
        builder.addScalar("mach_number", [this]() { return mach_number_; });
        builder.addScalar("dynamic_pressure_pa",
                          [this]() { return dynamic_pressure_pa_; });
        builder.addScalar("lift_coefficient",
                          [this]() { return coefficients_.lift_coefficient; });
        builder.addScalar("drag_coefficient",
                          [this]() { return coefficients_.drag_coefficient; });
        return builder.build();
    }

private:
    double currentMassKg() const {
        return continuous_mass_ ? continuous_mass_->getMassKg()
                                : constant_mass_->getMassKg();
    }

    gnc::forms::cartesian_3dof::Input currentInput() const {
        auto input = configured_input_;
        if (guidance_) {
            input.acceleration_mps2 +=
                guidance_->guidanceCommand3Dof().acceleration_command_nue_mps2;
        }
        last_input_ = input;
        return input;
    }

    void refreshDiagnostics(
        const gnc::forms::cartesian_3dof::Truth* truth) const {
        if (air_data_) {
            const auto& air_data = air_data_->airDataMeasurement3Dof();
            mach_number_ = air_data.mach_number;
            dynamic_pressure_pa_ = air_data.dynamic_pressure_pa;
        } else if (truth) {
            mach_number_ = truth->state.velocity_mps.norm() / 340.0;
            dynamic_pressure_pa_ = 0.0;
        } else {
            mach_number_ = 0.0;
            dynamic_pressure_pa_ = 0.0;
        }

        double angle_of_attack_rad = 0.0;
        if (actuator_) {
            angle_of_attack_rad =
                actuator_->actuatorState3Dof().angle_of_attack_rad;
        }
        coefficients_ =
            aero_->computeCoefficients(angle_of_attack_rad, 0.0, mach_number_);
        propulsion_force_n_ = propulsion_->getForceN();
        mass_kg_ = std::max(1.0e-9, currentMassKg());
        (void)currentInput();
    }

    gnc::vehicle::process::IGuidance3Dof* guidance_ = nullptr;
    gnc::vehicle::input::IAirData3Dof* air_data_ = nullptr;
    gnc::vehicle::output::IAeroModel* aero_ = nullptr;
    gnc::vehicle::output::IForceProvider* propulsion_ = nullptr;
    gnc::vehicle::output::IActuator3Dof* actuator_ = nullptr;
    gnc::vehicle::output::IConstantMass* constant_mass_ = nullptr;
    gnc::vehicle::output::IContinuousMass* continuous_mass_ = nullptr;
    std::string guidance_lookup_name_ = "guidance";
    std::string air_data_lookup_name_ = "air_data";
    std::string aero_lookup_name_ = "aero";
    std::string propulsion_lookup_name_ = "propulsion";
    std::string actuator_lookup_name_ = "actuator";
    std::string mass_lookup_name_ = "mass";
    gnc::forms::cartesian_3dof::Input configured_input_{};
    mutable gnc::forms::cartesian_3dof::Input last_input_{};
    mutable gnc::math::Vector3 propulsion_force_n_ = gnc::math::Vector3::Zero();
    mutable gnc::vehicle::output::AeroCoefficients coefficients_{};
    mutable double mass_kg_ = 1.0;
    mutable double mach_number_ = 0.0;
    mutable double dynamic_pressure_pa_ = 0.0;
};

} // namespace gnc::interactions::cartesian_3dof
