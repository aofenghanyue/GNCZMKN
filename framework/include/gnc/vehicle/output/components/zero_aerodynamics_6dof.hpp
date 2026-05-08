#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/common/interfaces/i_aerodynamic_assets_6dof.hpp"
#include "gnc/vehicle/output/interfaces/i_actuator_6dof.hpp"
#include "gnc/vehicle/output/interfaces/i_aerodynamics_6dof.hpp"

namespace gnc::vehicle::output {

class ZeroAerodynamics6Dof final : public gnc::core::ComponentBase,
                                   public IAerodynamics6Dof,
                                   public gnc::interfaces::IObservable {
public:
    ZeroAerodynamics6Dof() : ComponentBase("ZeroAerodynamics6Dof") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        aero_assets_lookup_name_ =
            reader.optionalString("aero_assets_lookup_name",
                                  aero_assets_lookup_name_);
        actuator_lookup_name_ =
            reader.optionalString("actuator_lookup_name", actuator_lookup_name_);
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(
            gnc::core::bind(aero_assets_, aero_assets_lookup_name_),
            gnc::core::bindIfPresent(actuator_, actuator_lookup_name_));
    }

    void update(double) override {
        gnc::vehicle::AeroQuery6Dof query;
        if (actuator_) {
            query.control_surfaces = actuator_->actuatorState6Dof();
        }
        last_coefficients_ = aero_assets_->sampleAeroCoefficients6Dof(query);
        force_moment_ = {};
    }

    gnc::vehicle::ForceMoment6Dof aerodynamicForceMoment6Dof() const override {
        return force_moment_;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addVector3("force_body", [this]() -> const gnc::math::Vector3& {
            return force_moment_.force_body_n;
        });
        builder.addVector3("moment_body", [this]() -> const gnc::math::Vector3& {
            return force_moment_.moment_body_nm;
        });
        builder.addScalar("coefficients.cx",
                          [this]() { return last_coefficients_.cx; });
        builder.addScalar("coefficients.cy",
                          [this]() { return last_coefficients_.cy; });
        builder.addScalar("coefficients.cz",
                          [this]() { return last_coefficients_.cz; });
        builder.addScalar("coefficients.cl",
                          [this]() { return last_coefficients_.cl; });
        builder.addScalar("coefficients.cm",
                          [this]() { return last_coefficients_.cm; });
        builder.addScalar("coefficients.cn",
                          [this]() { return last_coefficients_.cn; });
        return builder.build();
    }

private:
    gnc::vehicle::common::IAerodynamicAssets6Dof* aero_assets_ = nullptr;
    IActuator6Dof* actuator_ = nullptr;
    std::string aero_assets_lookup_name_ = "aero_assets";
    std::string actuator_lookup_name_ = "actuator";
    gnc::vehicle::AeroCoefficients6Dof last_coefficients_{};
    gnc::vehicle::ForceMoment6Dof force_moment_{};
};

} // namespace gnc::vehicle::output
