#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/environment/interfaces/i_atmosphere.hpp"
#include "gnc/environment/interfaces/i_gravity.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_input_provider.hpp"
#include "gnc/forms/local_spherical_3dof/internal/math.hpp"
#include "gnc/plugins/state_3dof/interfaces/i_acceleration_provider_3dof.hpp"
#include "gnc/plugins/state_3dof/interfaces/i_flight_command_provider_3dof.hpp"
#include "gnc/vehicle/common/interfaces/i_aero_model.hpp"
#include "gnc/vehicle/common/interfaces/i_constant_mass.hpp"
#include "gnc/vehicle/common/interfaces/i_continuous_mass.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace gnc::interactions::local_spherical_3dof {

class AeroPropulsive final
    : public gnc::core::ComponentBase,
      public gnc::forms::local_spherical_3dof::IInputProvider,
      public gnc::plugins::state_3dof::IAccelerationProvider3DOF {
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
        input.local_acceleration_nue_mps2 = computeLocalAcceleration(truth.state);
        return input;
    }

    gnc::math::Vector3 computeLocalAccelerationNue(
        const gnc::plugins::state_3dof::SovietSphericalState3DOF& state,
        double) const override {
        gnc::forms::local_spherical_3dof::State local_state;
        local_state.longitude_rad = state.longitude_rad;
        local_state.latitude_rad = state.latitude_rad;
        local_state.altitude_m = state.altitude_m;
        local_state.speed_mps = state.speed_mps;
        local_state.flight_path_angle_rad = state.flight_path_angle_rad;
        local_state.heading_angle_rad = state.heading_angle_rad;
        return computeLocalAcceleration(local_state);
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

    gnc::math::Vector3 computeLocalAcceleration(
        const gnc::forms::local_spherical_3dof::State& state) const {
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
            gnc::forms::local_spherical_3dof::internal::velocityBasisK1(state);
        const gnc::math::Vector3 k2 =
            gnc::forms::local_spherical_3dof::internal::velocityBasisK2(state);
        const gnc::math::Vector3 k3 =
            gnc::forms::local_spherical_3dof::internal::velocityBasisK3(state);

        return -drag_acceleration_m_per_s2 * k1 +
               lift_acceleration_m_per_s2 * std::cos(command.bank_angle_rad) * k2 -
               lift_acceleration_m_per_s2 * std::sin(command.bank_angle_rad) * k3 +
               gnc::math::Vector3(0.0, -gravity_m_per_s2, 0.0);
    }

    gnc::environment::IAtmosphere* atmosphere_ = nullptr;
    gnc::environment::IGravity* gravity_ = nullptr;
    gnc::vehicle::common::IAeroModel* aero_model_ = nullptr;
    gnc::plugins::state_3dof::IFlightCommandProvider3DOF* command_provider_ = nullptr;
    gnc::vehicle::common::IConstantMass* constant_mass_ = nullptr;
    gnc::vehicle::common::IContinuousMass* continuous_mass_ = nullptr;
};

} // namespace gnc::interactions::local_spherical_3dof
