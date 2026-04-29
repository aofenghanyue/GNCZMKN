#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/forms/cartesian_3dof/interfaces/i_input_provider.hpp"
#include "gnc/vehicle/output/interfaces/i_constant_mass.hpp"
#include "gnc/vehicle/output/interfaces/i_continuous_mass.hpp"
#include "gnc/vehicle/output/interfaces/i_force_provider.hpp"

#include <stdexcept>
#include <string>

namespace gnc::interactions::cartesian_3dof {

class ForceAccel final : public gnc::core::ComponentBase,
                         public gnc::forms::cartesian_3dof::IInputProvider {
public:
    ForceAccel() : ComponentBase("Cartesian3DoFForceAccel") {}

    void configure(const gnc::core::ConfigNode& config) override {
        configure(config, "config");
    }

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        force_lookup_name_ = reader.optionalString("force_lookup_name",
                                                   force_lookup_name_);
        mass_lookup_name_ = reader.optionalString("mass_lookup_name",
                                                  mass_lookup_name_);
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        force_provider_ =
            registry.requireByName<gnc::vehicle::output::IForceProvider>(
                force_lookup_name_);
        registry.bindAll(
            gnc::core::bindIfPresent(constant_mass_, mass_lookup_name_),
            gnc::core::bindIfPresent(continuous_mass_, mass_lookup_name_));

        if ((constant_mass_ != nullptr) == (continuous_mass_ != nullptr)) {
            throw std::runtime_error(
                "interaction.cartesian_3dof.force_accel requires exactly one "
                "mass provider named '" +
                mass_lookup_name_ +
                "' implementing either IConstantMass or IContinuousMass.");
        }
    }

    void update(double) override {}

    gnc::forms::cartesian_3dof::Input computeCartesian3DoFInput(
        const gnc::forms::cartesian_3dof::Truth&,
        double) const override {
        if (!force_provider_) {
            throw std::runtime_error(
                "interaction.cartesian_3dof.force_accel has no force provider.");
        }
        const double mass_kg = currentMassKg();
        if (mass_kg <= 0.0) {
            throw std::runtime_error(
                "interaction.cartesian_3dof.force_accel requires positive mass.");
        }

        gnc::forms::cartesian_3dof::Input input;
        input.acceleration_mps2 = force_provider_->getForceN() / mass_kg;
        return input;
    }

private:
    double currentMassKg() const {
        if (continuous_mass_) {
            return continuous_mass_->getMassKg();
        }
        return constant_mass_->getMassKg();
    }

    gnc::vehicle::output::IForceProvider* force_provider_ = nullptr;
    gnc::vehicle::output::IConstantMass* constant_mass_ = nullptr;
    gnc::vehicle::output::IContinuousMass* continuous_mass_ = nullptr;
    std::string force_lookup_name_ = "force";
    std::string mass_lookup_name_ = "mass";
};

} // namespace gnc::interactions::cartesian_3dof
