#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/environment/interfaces/i_gravity.hpp"
#include "gnc/forms/local_spherical_6dof/interfaces/i_input_provider.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/output/interfaces/i_aerodynamics_6dof.hpp"
#include "gnc/vehicle/output/interfaces/i_mass_properties_6dof.hpp"
#include "gnc/vehicle/output/interfaces/i_propulsion_6dof.hpp"

#include <algorithm>
#include <string>

namespace gnc::interactions::local_spherical_6dof {

class StandardClosure final
    : public gnc::core::ComponentBase,
      public gnc::forms::local_spherical_6dof::IInputProvider,
      public gnc::interfaces::IObservable {
public:
    StandardClosure() : ComponentBase("LocalSpherical6DoFStandardClosure") {}

    void configure(const gnc::core::ConfigNode& config) override {
        configure(config, "config");
    }

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        gravity_lookup_name_ =
            reader.optionalString("gravity_lookup_name", gravity_lookup_name_);
        aero_lookup_name_ =
            reader.optionalString("aero_lookup_name", aero_lookup_name_);
        propulsion_lookup_name_ =
            reader.optionalString("propulsion_lookup_name", propulsion_lookup_name_);
        mass_lookup_name_ =
            reader.optionalString("mass_lookup_name", mass_lookup_name_);

        const auto acceleration =
            reader.optionalDoubleArray("local_acceleration_nue_mps2",
                                       {0.0, 0.0, 0.0},
                                       3);
        placeholder_input_.local_acceleration_nue_mps2 =
            gnc::math::Vector3(acceleration[0], acceleration[1], acceleration[2]);

        const auto angular_acceleration =
            reader.optionalDoubleArray("angular_acceleration_body_radps2",
                                       {0.0, 0.0, 0.0},
                                       3);
        placeholder_input_.angular_acceleration_body_radps2 =
            gnc::math::Vector3(angular_acceleration[0],
                               angular_acceleration[1],
                               angular_acceleration[2]);
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(
            gnc::core::bindIfPresent(gravity_, gravity_lookup_name_),
            gnc::core::bind(aero_, aero_lookup_name_),
            gnc::core::bind(propulsion_, propulsion_lookup_name_),
            gnc::core::bind(mass_, mass_lookup_name_));
    }

    void initialize() override { update(0.0); }

    void update(double) override { refreshDiagnostics(nullptr); }

    gnc::forms::local_spherical_6dof::Input computeLocalSpherical6DoFInput(
        const gnc::forms::local_spherical_6dof::Truth& truth,
        double) const override {
        refreshDiagnostics(&truth);
        return placeholder_input_;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addVector3("total_force_body",
                           [this]() -> const gnc::math::Vector3& {
                               return total_force_body_n_;
                           });
        builder.addVector3("total_moment_body",
                           [this]() -> const gnc::math::Vector3& {
                               return total_moment_body_nm_;
                           });
        builder.addVector3("local_acceleration_nue",
                           [this]() -> const gnc::math::Vector3& {
                               return placeholder_input_.local_acceleration_nue_mps2;
                           });
        builder.addVector3("angular_acceleration_body",
                           [this]() -> const gnc::math::Vector3& {
                               return placeholder_input_.angular_acceleration_body_radps2;
                           });
        builder.addScalar("mass_kg", [this]() { return mass_kg_; });
        builder.addScalar("gravity_mps2", [this]() { return gravity_mps2_; });
        return builder.build();
    }

private:
    void refreshDiagnostics(
        const gnc::forms::local_spherical_6dof::Truth* truth) const {
        const auto aero_force_moment = aero_->aerodynamicForceMoment6Dof();
        const auto propulsion_force_moment =
            propulsion_->propulsionForceMoment6Dof();
        total_force_body_n_ =
            aero_force_moment.force_body_n + propulsion_force_moment.force_body_n;
        total_moment_body_nm_ =
            aero_force_moment.moment_body_nm + propulsion_force_moment.moment_body_nm;

        const auto mass_properties = mass_->massProperties6Dof();
        mass_kg_ = std::max(1.0e-9, mass_properties.mass_kg);
        if (gravity_ && truth) {
            gravity_mps2_ = gravity_->getGravityMagnitude(truth->state.altitude_m);
        } else if (gravity_) {
            gravity_mps2_ = gravity_->getSeaLevelGravity();
        } else {
            gravity_mps2_ = 0.0;
        }
    }

    gnc::environment::IGravity* gravity_ = nullptr;
    gnc::vehicle::output::IAerodynamics6Dof* aero_ = nullptr;
    gnc::vehicle::output::IPropulsion6Dof* propulsion_ = nullptr;
    gnc::vehicle::output::IMassProperties6Dof* mass_ = nullptr;
    std::string gravity_lookup_name_ = "env.gravity";
    std::string aero_lookup_name_ = "aero";
    std::string propulsion_lookup_name_ = "propulsion";
    std::string mass_lookup_name_ = "mass";
    gnc::forms::local_spherical_6dof::Input placeholder_input_{};
    mutable gnc::math::Vector3 total_force_body_n_ = gnc::math::Vector3::Zero();
    mutable gnc::math::Vector3 total_moment_body_nm_ = gnc::math::Vector3::Zero();
    mutable double mass_kg_ = 1.0;
    mutable double gravity_mps2_ = 0.0;
};

} // namespace gnc::interactions::local_spherical_6dof
