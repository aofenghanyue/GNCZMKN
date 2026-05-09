#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
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

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace cavh::components {

class AeroPropulsiveInteraction3Dof final
    : public gnc::core::ComponentBase,
      public gnc::forms::local_spherical_3dof::IInputProvider,
      public gnc::interfaces::IObservable {
public:
    AeroPropulsiveInteraction3Dof()
        : ComponentBase("CavhAeroPropulsiveInteraction3Dof") {}

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
        actuator_lookup_name_ =
            reader.optionalString("actuator_lookup_name", actuator_lookup_name_);
        mass_lookup_name_ =
            reader.optionalString("mass_lookup_name", mass_lookup_name_);
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(
            gnc::core::bind(atmosphere_, atmosphere_lookup_name_),
            gnc::core::bind(gravity_, gravity_lookup_name_),
            gnc::core::bind(aero_, aero_lookup_name_),
            gnc::core::bind(actuator_, actuator_lookup_name_),
            gnc::core::bindIfPresent(constant_mass_, mass_lookup_name_),
            gnc::core::bindIfPresent(continuous_mass_, mass_lookup_name_));

        if ((constant_mass_ != nullptr) == (continuous_mass_ != nullptr)) {
            throw std::runtime_error(
                "cavh.interaction.local_spherical_3dof.aero_propulsive requires exactly one mass provider implementing IConstantMass or IContinuousMass.");
        }
    }

    void update(double) override {}

    gnc::forms::local_spherical_3dof::Input computeLocalSpherical3DoFInput(
        const gnc::forms::local_spherical_3dof::Truth& truth,
        double) const override {
        gnc::forms::local_spherical_3dof::Input input;
        input.local_acceleration_nue_mps2 = computeLocalAccelerationNue(truth);
        return input;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields()
        const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addVector3("local_acceleration_nue",
                           [this]() -> const gnc::math::Vector3& {
                               return local_acceleration_nue_mps2_;
                           });
        builder.addScalar("angle_of_attack_rad",
                          [this]() { return angle_of_attack_rad_; });
        builder.addScalar("bank_angle_rad",
                          [this]() { return bank_angle_rad_; });
        builder.addScalar("mach_number", [this]() { return mach_number_; });
        builder.addScalar("dynamic_pressure_pa",
                          [this]() { return dynamic_pressure_pa_; });
        builder.addScalar("lift_coefficient",
                          [this]() { return coefficients_.lift_coefficient; });
        builder.addScalar("drag_coefficient",
                          [this]() { return coefficients_.drag_coefficient; });
        builder.addScalar("mass_kg", [this]() { return mass_kg_; });
        builder.addScalar("gravity_mps2", [this]() { return gravity_mps2_; });
        return builder.build();
    }

private:
    double currentMassKg() const {
        return continuous_mass_ ? continuous_mass_->getMassKg()
                                : constant_mass_->getMassKg();
    }

    gnc::math::Vector3 computeLocalAccelerationNue(
        const gnc::forms::local_spherical_3dof::Truth& truth) const {
        const auto& state = truth.state;
        const auto atmosphere_sample = atmosphere_->sample(state.altitude_m);
        const double speed_of_sound =
            std::max(1.0, atmosphere_sample.speed_of_sound_mps);
        mach_number_ = state.speed_mps / speed_of_sound;
        dynamic_pressure_pa_ = 0.5 * atmosphere_sample.density_kg_per_m3 *
                               state.speed_mps * state.speed_mps;

        const auto& actuator = actuator_->actuatorState3Dof();
        angle_of_attack_rad_ = actuator.angle_of_attack_rad;
        bank_angle_rad_ = actuator.bank_angle_rad;
        coefficients_ =
            aero_->computeCoefficients(angle_of_attack_rad_, 0.0, mach_number_);

        mass_kg_ = std::max(1.0e-9, currentMassKg());
        gravity_mps2_ = gravity_->getGravityMagnitude(state.altitude_m);
        const double area_m2 = aero_->getReferenceArea();
        const double lift_acceleration_mps2 =
            dynamic_pressure_pa_ * area_m2 *
            coefficients_.lift_coefficient / mass_kg_;
        const double drag_acceleration_mps2 =
            dynamic_pressure_pa_ * area_m2 *
            coefficients_.drag_coefficient / mass_kg_;

        local_acceleration_nue_mps2_ =
            -drag_acceleration_mps2 * truth.drag_axis_nue +
            lift_acceleration_mps2 * std::cos(bank_angle_rad_) *
                truth.lift_up_axis_nue -
            lift_acceleration_mps2 * std::sin(bank_angle_rad_) *
                truth.lift_side_axis_nue +
            gnc::math::Vector3(0.0, -gravity_mps2_, 0.0);
        return local_acceleration_nue_mps2_;
    }

    gnc::environment::IAtmosphere* atmosphere_ = nullptr;
    gnc::environment::IGravity* gravity_ = nullptr;
    gnc::vehicle::output::IAeroModel* aero_ = nullptr;
    gnc::vehicle::output::IActuator3Dof* actuator_ = nullptr;
    gnc::vehicle::output::IConstantMass* constant_mass_ = nullptr;
    gnc::vehicle::output::IContinuousMass* continuous_mass_ = nullptr;
    std::string atmosphere_lookup_name_ = "env.atmosphere";
    std::string gravity_lookup_name_ = "env.gravity";
    std::string aero_lookup_name_ = "aero";
    std::string actuator_lookup_name_ = "actuator";
    std::string mass_lookup_name_ = "mass";
    mutable gnc::math::Vector3 local_acceleration_nue_mps2_ =
        gnc::math::Vector3::Zero();
    mutable gnc::vehicle::output::AeroCoefficients coefficients_{};
    mutable double angle_of_attack_rad_ = 0.0;
    mutable double bank_angle_rad_ = 0.0;
    mutable double mach_number_ = 0.0;
    mutable double dynamic_pressure_pa_ = 0.0;
    mutable double mass_kg_ = 1.0;
    mutable double gravity_mps2_ = 0.0;
};

} // namespace cavh::components

GNC_REGISTER_COMPONENT_TYPE(
    "cavh.interaction.local_spherical_3dof.aero_propulsive",
    cavh::components::AeroPropulsiveInteraction3Dof,
    ::gnc::core::ComponentPackageRole::Interaction,
    ::gnc::core::ExecutionStage::Interaction,
    "local_spherical_3dof",
    ::gnc::forms::local_spherical_3dof::IInputProvider,
    ::gnc::interfaces::IObservable)
