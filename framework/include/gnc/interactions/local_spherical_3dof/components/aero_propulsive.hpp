#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/environment/interfaces/i_atmosphere.hpp"
#include "gnc/environment/interfaces/i_gravity.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_input_provider.hpp"
#include "gnc/vehicle/common/interfaces/i_aero_model.hpp"
#include "gnc/vehicle/common/interfaces/i_constant_mass.hpp"
#include "gnc/vehicle/common/interfaces/i_continuous_mass.hpp"
#include "gnc/vehicle/process/interfaces/i_aero_guidance_provider.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace gnc::interactions::local_spherical_3dof {

class AeroPropulsive final
    : public gnc::core::ComponentBase,
      public gnc::forms::local_spherical_3dof::IInputProvider {
public:
    AeroPropulsive() : ComponentBase("LocalSpherical3DoFAeroPropulsive") {}

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(
            gnc::core::bind(atmosphere_, "env.atmosphere"),
            gnc::core::bind(gravity_, "env.gravity"),
            gnc::core::bind(aero_model_, "aero"),
            gnc::core::bindIfPresent(command_provider_, "guidance"),
            gnc::core::bindIfPresent(constant_mass_, "mass"),
            gnc::core::bindIfPresent(continuous_mass_, "mass"));

        if ((constant_mass_ != nullptr) == (continuous_mass_ != nullptr)) {
            throw std::runtime_error(
                "interaction.local_spherical_3dof.aero_propulsive requires exactly "
                "one mass provider named 'mass' implementing either "
                "IConstantMass or IContinuousMass.");
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

private:
    gnc::vehicle::process::AeroGuidanceCommand currentCommand() const {
        if (command_provider_ && command_provider_->isGuidanceActive()) {
            return command_provider_->getAeroGuidanceCommand();
        }
        return {};
    }

    double currentMassKg() const {
        if (continuous_mass_) {
            return continuous_mass_->getMassKg();
        }
        return constant_mass_->getMassKg();
    }

public:
    gnc::math::Vector3 computeLocalAccelerationNue(
        const gnc::forms::local_spherical_3dof::Truth& truth) const {
        const auto& state = truth.state;
        const auto atmosphere_sample = atmosphere_->sample(state.altitude_m);
        const double speed_of_sound =
            std::max(1.0, atmosphere_sample.speed_of_sound_mps);
        const double mach_number = state.speed_mps / speed_of_sound;
        const auto command = currentCommand();
        const auto coefficients =
            aero_model_->computeCoefficients(command.angle_of_attack_rad, 0.0, mach_number);

        const double mass_kg = currentMassKg();
        const double dynamic_pressure_pa =
            0.5 * atmosphere_sample.density_kg_per_m3 * state.speed_mps * state.speed_mps;
        const double lift_acceleration_m_per_s2 =
            dynamic_pressure_pa * aero_model_->getReferenceArea() *
            coefficients.lift_coefficient / mass_kg;
        const double drag_acceleration_m_per_s2 =
            dynamic_pressure_pa * aero_model_->getReferenceArea() *
            coefficients.drag_coefficient / mass_kg;
        const double gravity_m_per_s2 =
            gravity_->getGravityMagnitude(state.altitude_m);

        return -drag_acceleration_m_per_s2 * truth.drag_axis_nue +
               lift_acceleration_m_per_s2 * std::cos(command.bank_angle_rad) *
                   truth.lift_up_axis_nue -
               lift_acceleration_m_per_s2 * std::sin(command.bank_angle_rad) *
                   truth.lift_side_axis_nue +
               gnc::math::Vector3(0.0, -gravity_m_per_s2, 0.0);
    }

private:
    gnc::environment::IAtmosphere* atmosphere_ = nullptr;
    gnc::environment::IGravity* gravity_ = nullptr;
    gnc::vehicle::common::IAeroModel* aero_model_ = nullptr;
    gnc::vehicle::process::IAeroGuidanceProvider* command_provider_ = nullptr;
    gnc::vehicle::common::IConstantMass* constant_mass_ = nullptr;
    gnc::vehicle::common::IContinuousMass* continuous_mass_ = nullptr;
};

} // namespace gnc::interactions::local_spherical_3dof
