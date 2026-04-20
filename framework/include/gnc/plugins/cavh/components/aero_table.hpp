#pragma once

#include "gnc/common/math/eigen_types.hpp"
#include "gnc/common/math/interp.hpp"
#include "gnc/core/component_base.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/plugins/aero/interfaces/i_aero_model.hpp"

#include <cmath>
#include <vector>

namespace gnc::plugins::cavh {

class AeroTable final : public gnc::core::ComponentBase,
                        public gnc::plugins::aero::IAeroModel,
                        public gnc::interfaces::IObservable {
public:
    AeroTable()
        : ComponentBase("CavhAeroTable"),
          lift_table_(alphaBreaksRad(), machBreaks(), liftCoefficientData()),
          drag_table_(alphaBreaksRad(), machBreaks(), dragCoefficientData()) {}

    void configure(const gnc::core::ConfigNode& config) override {
        reference_area_m2_ =
            config["reference_area_m2"].asDouble(reference_area_m2_);
        reference_length_m_ =
            config["reference_length_m"].asDouble(reference_length_m_);
    }

    void update(double) override {}

    gnc::plugins::aero::AeroCoefficients computeCoefficients(double angle_of_attack_rad,
                                                             double,
                                                             double mach_number) const override {
        gnc::plugins::aero::AeroCoefficients coefficients;
        coefficients.lift_coefficient = lift_table_.lookup(angle_of_attack_rad, mach_number);
        coefficients.drag_coefficient = drag_table_.lookup(angle_of_attack_rad, mach_number);

        last_angle_of_attack_rad_ = angle_of_attack_rad;
        last_mach_number_ = mach_number;
        last_coefficients_ = coefficients;
        return coefficients;
    }

    double getReferenceArea() const override { return reference_area_m2_; }
    double getReferenceLength() const override { return reference_length_m_; }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("angle_of_attack_rad",
                          [this]() { return last_angle_of_attack_rad_; });
        builder.addScalar("mach_number", [this]() { return last_mach_number_; });
        builder.addScalar("lift_coefficient",
                          [this]() { return last_coefficients_.lift_coefficient; });
        builder.addScalar("drag_coefficient",
                          [this]() { return last_coefficients_.drag_coefficient; });
        builder.addScalar(
            "lift_to_drag",
            [this]() {
                return std::abs(last_coefficients_.drag_coefficient) > 1.0e-9
                           ? last_coefficients_.lift_coefficient /
                                 last_coefficients_.drag_coefficient
                           : 0.0;
            });
        return builder.build();
    }

private:
    static std::vector<double> machBreaks() {
        return {3.5, 5.0, 8.0, 10.0, 15.0, 20.0, 23.0};
    }

    static std::vector<double> alphaBreaksRad() {
        return {10.0 * gnc::math::constants::DEG_TO_RAD,
                15.0 * gnc::math::constants::DEG_TO_RAD,
                20.0 * gnc::math::constants::DEG_TO_RAD};
    }

    static std::vector<std::vector<double>> liftCoefficientData() {
        return {{0.4500, 0.4250, 0.4000, 0.3800, 0.3700, 0.3600, 0.3500},
                {0.7400, 0.7000, 0.6700, 0.6300, 0.6000, 0.5700, 0.5570},
                {1.0500, 1.0000, 0.9500, 0.9000, 0.8500, 0.8000, 0.7800}};
    }

    static std::vector<std::vector<double>> dragCoefficientData() {
        return {{0.2045, 0.1700, 0.1290, 0.1090, 0.1090, 0.1090, 0.1090},
                {0.2960, 0.2630, 0.2240, 0.1970, 0.1950, 0.1920, 0.1920},
                {0.4770, 0.4230, 0.3540, 0.3100, 0.3050, 0.3000, 0.3000}};
    }

    gnc::math::LookupTable2D lift_table_;
    gnc::math::LookupTable2D drag_table_;
    double reference_area_m2_ = 750.0 * 0.00064516;
    double reference_length_m_ = 3.0;
    mutable double last_angle_of_attack_rad_ = 0.0;
    mutable double last_mach_number_ = 0.0;
    mutable gnc::plugins::aero::AeroCoefficients last_coefficients_{};
};

} // namespace gnc::plugins::cavh
