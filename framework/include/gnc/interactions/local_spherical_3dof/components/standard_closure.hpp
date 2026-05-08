#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/environment/interfaces/i_atmosphere.hpp"
#include "gnc/environment/interfaces/i_gravity.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_input_provider.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/output/interfaces/i_actuator_3dof.hpp"
#include "gnc/vehicle/output/interfaces/i_aero_model.hpp"
#include "gnc/vehicle/output/interfaces/i_constant_mass.hpp"
#include "gnc/vehicle/output/interfaces/i_continuous_mass.hpp"
#include "gnc/vehicle/output/interfaces/i_force_provider.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace gnc::interactions::local_spherical_3dof {

class StandardClosure final
    : public gnc::core::ComponentBase,
      public gnc::forms::local_spherical_3dof::IInputProvider,
      public gnc::interfaces::IObservable {
public:
    StandardClosure() : ComponentBase("LocalSpherical3DoFStandardClosure") {}

    void configure(const gnc::core::ConfigNode& config) override {
        configure(config, "config");
    }

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        atmosphere_lookup_name_ =
            reader.optionalString("atmosphere_lookup_name",
                                  atmosphere_lookup_name_);
        gravity_lookup_name_ =
            reader.optionalString("gravity_lookup_name", gravity_lookup_name_);
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
            reader.optionalDoubleArray("local_acceleration_nue_mps2",
                                       {0.0, 0.0, 0.0},
                                       3);
        placeholder_input_.local_acceleration_nue_mps2 =
            gnc::math::Vector3(acceleration[0], acceleration[1], acceleration[2]);
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(
            gnc::core::bindIfPresent(atmosphere_, atmosphere_lookup_name_),
            gnc::core::bindIfPresent(gravity_, gravity_lookup_name_),
            gnc::core::bind(aero_, aero_lookup_name_),
            gnc::core::bind(propulsion_, propulsion_lookup_name_),
            gnc::core::bindIfPresent(actuator_, actuator_lookup_name_),
            gnc::core::bindIfPresent(constant_mass_, mass_lookup_name_),
            gnc::core::bindIfPresent(continuous_mass_, mass_lookup_name_));

        if ((constant_mass_ != nullptr) == (continuous_mass_ != nullptr)) {
            throw std::runtime_error(
                "interaction.local_spherical_3dof.standard requires exactly "
                "one mass provider named 'mass' implementing either "
                "IConstantMass or IContinuousMass.");
        }
    }

    void initialize() override { update(0.0); }

    void update(double) override { refreshDiagnostics(nullptr); }

    gnc::forms::local_spherical_3dof::Input computeLocalSpherical3DoFInput(
        const gnc::forms::local_spherical_3dof::Truth& truth,
        double) const override {
        refreshDiagnostics(&truth);
        return placeholder_input_;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addVector3("propulsion_force_n",
                           [this]() -> const gnc::math::Vector3& {
                               return propulsion_force_n_;
                           });
        builder.addVector3("local_acceleration_nue",
                           [this]() -> const gnc::math::Vector3& {
                               return placeholder_input_.local_acceleration_nue_mps2;
                           });
        builder.addScalar("mass_kg", [this]() { return mass_kg_; });
        builder.addScalar("gravity_mps2", [this]() { return gravity_mps2_; });
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

    void refreshDiagnostics(
        const gnc::forms::local_spherical_3dof::Truth* truth) const {
        const double altitude_m = truth ? truth->state.altitude_m : 0.0;
        const double speed_mps = truth ? truth->state.speed_mps : 0.0;
        const auto atmosphere_sample =
            atmosphere_ ? atmosphere_->sample(altitude_m)
                        : gnc::environment::AtmosphereSample{};
        const double speed_of_sound =
            std::max(1.0, atmosphere_sample.speed_of_sound_mps);
        mach_number_ = speed_mps / speed_of_sound;
        dynamic_pressure_pa_ =
            0.5 * atmosphere_sample.density_kg_per_m3 * speed_mps * speed_mps;

        double angle_of_attack_rad = 0.0;
        double bank_angle_rad = 0.0;
        if (actuator_) {
            const auto& actuator = actuator_->actuatorState3Dof();
            angle_of_attack_rad = actuator.angle_of_attack_rad;
            bank_angle_rad = actuator.bank_angle_rad;
        }
        coefficients_ =
            aero_->computeCoefficients(angle_of_attack_rad, 0.0, mach_number_);
        bank_angle_rad_ = bank_angle_rad;
        propulsion_force_n_ = propulsion_->getForceN();
        mass_kg_ = std::max(1.0e-9, currentMassKg());
        gravity_mps2_ =
            gravity_ ? gravity_->getGravityMagnitude(altitude_m) : 0.0;
    }

    gnc::environment::IAtmosphere* atmosphere_ = nullptr;
    gnc::environment::IGravity* gravity_ = nullptr;
    gnc::vehicle::output::IAeroModel* aero_ = nullptr;
    gnc::vehicle::output::IForceProvider* propulsion_ = nullptr;
    gnc::vehicle::output::IActuator3Dof* actuator_ = nullptr;
    gnc::vehicle::output::IConstantMass* constant_mass_ = nullptr;
    gnc::vehicle::output::IContinuousMass* continuous_mass_ = nullptr;
    std::string atmosphere_lookup_name_ = "env.atmosphere";
    std::string gravity_lookup_name_ = "env.gravity";
    std::string aero_lookup_name_ = "aero";
    std::string propulsion_lookup_name_ = "propulsion";
    std::string actuator_lookup_name_ = "actuator";
    std::string mass_lookup_name_ = "mass";
    gnc::forms::local_spherical_3dof::Input placeholder_input_{};
    mutable gnc::math::Vector3 propulsion_force_n_ = gnc::math::Vector3::Zero();
    mutable gnc::vehicle::output::AeroCoefficients coefficients_{};
    mutable double bank_angle_rad_ = 0.0;
    mutable double mass_kg_ = 1.0;
    mutable double gravity_mps2_ = 0.0;
    mutable double mach_number_ = 0.0;
    mutable double dynamic_pressure_pa_ = 0.0;
};

} // namespace gnc::interactions::local_spherical_3dof
