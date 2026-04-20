#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/plugins/aero/interfaces/i_aero_model.hpp"
#include "gnc/plugins/environment/interfaces/i_atmosphere.hpp"
#include "gnc/plugins/environment/interfaces/i_gravity.hpp"
#include "gnc/plugins/mass/interfaces/i_constant_mass.hpp"
#include "gnc/plugins/mass/interfaces/i_continuous_mass.hpp"
#include "gnc/plugins/state_3dof/interfaces/i_acceleration_provider_3dof.hpp"
#include "gnc/plugins/state_3dof/interfaces/i_flight_command_provider_3dof.hpp"
#include "gnc/plugins/state_3dof/internal/soviet_spherical_3dof_math.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace gnc::plugins::state_3dof_bridge {

class ForceToLocalAccelerationSoviet final
    : public gnc::core::ComponentBase,
      public gnc::plugins::state_3dof::IAccelerationProvider3DOF {
public:
    ForceToLocalAccelerationSoviet()
        : ComponentBase("ForceToLocalAccelerationSoviet") {}

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
                "state_3dof_bridge.force_to_local_acceleration_soviet requires "
                "exactly one mass provider named 'mass' implementing either "
                "IConstantMass or IContinuousMass.");
        }
    }

    void update(double) override {}

    gnc::math::Vector3 computeLocalAccelerationNue(
        const gnc::plugins::state_3dof::SovietSphericalState3DOF& state,
        double) const override {
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

        const gnc::math::Vector3 k1 =
            gnc::plugins::state_3dof::internal::velocityBasisK1(state);
        const gnc::math::Vector3 k2 =
            gnc::plugins::state_3dof::internal::velocityBasisK2(state);
        const gnc::math::Vector3 k3 =
            gnc::plugins::state_3dof::internal::velocityBasisK3(state);

        return -drag_acceleration_m_per_s2 * k1 +
               lift_acceleration_m_per_s2 * std::cos(command.bank_angle_rad) * k2 -
               lift_acceleration_m_per_s2 * std::sin(command.bank_angle_rad) * k3 +
               gnc::math::Vector3(0.0, -gravity_m_per_s2, 0.0);
    }

private:
    gnc::plugins::state_3dof::FlightCommand3DOF currentCommand() const {
        if (command_provider_ && command_provider_->isActive()) {
            return command_provider_->getFlightCommand();
        }
        return {};
    }

    double currentMassKg() const {
        if (continuous_mass_) {
            return continuous_mass_->getMassKg();
        }
        return constant_mass_->getMassKg();
    }

    gnc::plugins::environment::IAtmosphere* atmosphere_ = nullptr;
    gnc::plugins::environment::IGravity* gravity_ = nullptr;
    gnc::plugins::aero::IAeroModel* aero_model_ = nullptr;
    gnc::plugins::state_3dof::IFlightCommandProvider3DOF* command_provider_ = nullptr;
    gnc::plugins::mass::IConstantMass* constant_mass_ = nullptr;
    gnc::plugins::mass::IContinuousMass* continuous_mass_ = nullptr;
};

} // namespace gnc::plugins::state_3dof_bridge
