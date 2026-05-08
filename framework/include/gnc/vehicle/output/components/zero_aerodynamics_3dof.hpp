#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/common/interfaces/i_aerodynamic_assets_3dof.hpp"
#include "gnc/vehicle/output/interfaces/i_actuator_3dof.hpp"
#include "gnc/vehicle/output/interfaces/i_aero_model.hpp"

namespace gnc::vehicle::output {

class ZeroAerodynamics3Dof final : public gnc::core::ComponentBase,
                                   public IAeroModel,
                                   public gnc::interfaces::IObservable {
public:
    ZeroAerodynamics3Dof() : ComponentBase("ZeroAerodynamics3Dof") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        assets_lookup_name_ =
            reader.optionalString("assets_lookup_name", assets_lookup_name_);
        actuator_lookup_name_ =
            reader.optionalString("actuator_lookup_name", actuator_lookup_name_);
        reference_area_m2_ =
            reader.optionalDouble("reference_area_m2", reference_area_m2_);
        reference_length_m_ =
            reader.optionalDouble("reference_length_m", reference_length_m_);
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(
            gnc::core::bind(assets_, assets_lookup_name_),
            gnc::core::bindIfPresent(actuator_, actuator_lookup_name_));
    }

    void initialize() override { update(0.0); }

    void update(double) override {
        double alpha = 0.0;
        if (actuator_) {
            alpha = actuator_->actuatorState3Dof().angle_of_attack_rad;
        }
        (void)computeCoefficients(alpha, 0.0, last_query_.mach_number);
    }

    AeroCoefficients computeCoefficients(double angle_of_attack_rad,
                                         double sideslip_angle_rad,
                                         double mach_number) const override {
        last_query_.angle_of_attack_rad = angle_of_attack_rad;
        last_query_.sideslip_angle_rad = sideslip_angle_rad;
        last_query_.mach_number = mach_number;
        last_coefficients_ = assets_->sampleAeroCoefficients3Dof(last_query_);
        return last_coefficients_;
    }

    double getReferenceArea() const override { return reference_area_m2_; }
    double getReferenceLength() const override { return reference_length_m_; }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("angle_of_attack_rad",
                          [this]() { return last_query_.angle_of_attack_rad; });
        builder.addScalar("sideslip_angle_rad",
                          [this]() { return last_query_.sideslip_angle_rad; });
        builder.addScalar("mach_number",
                          [this]() { return last_query_.mach_number; });
        builder.addScalar("lift_coefficient",
                          [this]() { return last_coefficients_.lift_coefficient; });
        builder.addScalar("drag_coefficient",
                          [this]() { return last_coefficients_.drag_coefficient; });
        return builder.build();
    }

private:
    gnc::vehicle::common::IAerodynamicAssets3Dof* assets_ = nullptr;
    IActuator3Dof* actuator_ = nullptr;
    std::string assets_lookup_name_ = "aero_assets";
    std::string actuator_lookup_name_ = "actuator";
    double reference_area_m2_ = 1.0;
    double reference_length_m_ = 1.0;
    mutable gnc::vehicle::common::AeroQuery3Dof last_query_{};
    mutable AeroCoefficients last_coefficients_{};
};

} // namespace gnc::vehicle::output
