#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/common/assets/json_asset_loader.hpp"
#include "gnc/vehicle/output/interfaces/i_aero_model.hpp"

#include <cmath>

namespace gnc::vehicle::output {

class SimplePolynomialAero final : public gnc::core::ComponentBase,
                                   public IAeroModel,
                                   public gnc::interfaces::IObservable {
public:
    SimplePolynomialAero() : ComponentBase("SimplePolynomialAero") {}

    void configure(const gnc::core::ConfigNode& config) override {
        const auto source =
            gnc::vehicle::common::assets::loadConfiguredJsonAsset(
                config,
                "aero.simple_polynomial");
        if (source.isNull()) {
            return;
        }
        lift_offset_ = source["lift_offset"].asDouble(lift_offset_);
        lift_slope_per_rad_ =
            source["lift_slope_per_rad"].asDouble(lift_slope_per_rad_);
        drag_zero_ = source["drag_zero"].asDouble(drag_zero_);
        drag_quadratic_ = source["drag_quadratic"].asDouble(drag_quadratic_);
        reference_area_m2_ = source["reference_area_m2"].asDouble(reference_area_m2_);
        reference_length_m_ =
            source["reference_length_m"].asDouble(reference_length_m_);
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
