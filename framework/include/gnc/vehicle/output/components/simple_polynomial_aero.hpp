#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/common/assets/json_asset_loader.hpp"
#include "gnc/vehicle/output/interfaces/i_aero_model.hpp"

#include <cmath>
#include <string>

namespace gnc::vehicle::output {

class SimplePolynomialAero final : public gnc::core::ComponentBase,
                                   public IAeroModel,
                                   public gnc::interfaces::IObservable {
public:
    SimplePolynomialAero() : ComponentBase("SimplePolynomialAero") {}

    void configure(const gnc::core::ConfigNode& config) override {
        configure(config, "config");
    }

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        const auto source =
            gnc::vehicle::common::assets::loadConfiguredJsonAsset(
                config,
                "aero.simple_polynomial",
                config_path);
        const bool using_asset_file =
            gnc::vehicle::common::assets::hasConfiguredJsonAssetFile(config,
                                                                     config_path);
        const std::string source_path =
            using_asset_file
                ? "aero.simple_polynomial asset '" +
                      gnc::vehicle::common::assets::resolveConfiguredJsonAssetPath(config,
                                                                                  config_path)
                          .generic_string() +
                      "'"
                : config_path;
        gnc::core::ConfigReader reader(source, source_path);
        lift_offset_ = reader.requiredDouble("lift_offset");
        lift_slope_per_rad_ = reader.requiredDouble("lift_slope_per_rad");
        drag_zero_ = reader.requiredDouble("drag_zero");
        drag_quadratic_ = reader.requiredDouble("drag_quadratic");
        reference_area_m2_ = reader.requiredDouble("reference_area_m2");
        reference_length_m_ = reader.requiredDouble("reference_length_m");
        reader.validateNoUnknownKeys();
    }

    void update(double) override {}

    AeroCoefficients computeCoefficients(double angle_of_attack_rad,
                                         double,
                                         double mach_number) const override {
        AeroCoefficients coefficients;
        coefficients.lift_coefficient =
            lift_offset_ + lift_slope_per_rad_ * angle_of_attack_rad;
        coefficients.drag_coefficient =
            drag_zero_ + drag_quadratic_ * angle_of_attack_rad * angle_of_attack_rad +
            0.015 * std::max(0.0, mach_number - 5.0);
        current_coefficients_ = coefficients;
        return coefficients;
    }

    double getReferenceArea() const override { return reference_area_m2_; }
    double getReferenceLength() const override { return reference_length_m_; }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("lift_coefficient",
                          [this]() { return current_coefficients_.lift_coefficient; });
        builder.addScalar("drag_coefficient",
                          [this]() { return current_coefficients_.drag_coefficient; });
        builder.addScalar(
            "lift_to_drag",
            [this]() {
                return std::abs(current_coefficients_.drag_coefficient) > 1e-9
                           ? current_coefficients_.lift_coefficient /
                                 current_coefficients_.drag_coefficient
                           : 0.0;
            });
        return builder.build();
    }

private:
    double lift_offset_ = 0.0;
    double lift_slope_per_rad_ = 1.6;
    double drag_zero_ = 0.08;
    double drag_quadratic_ = 1.25;
    double reference_area_m2_ = 0.48;
    double reference_length_m_ = 2.5;
    mutable AeroCoefficients current_coefficients_{};
};

} // namespace gnc::vehicle::output
