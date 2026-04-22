#pragma once

#include "gnc/common/math/eigen_types.hpp"
#include "gnc/common/math/interp.hpp"
#include "gnc/core/component_base.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/common/assets/json_asset_loader.hpp"
#include "gnc/vehicle/output/interfaces/i_aero_model.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace gnc::vehicle::output {

class Table2DAero final : public gnc::core::ComponentBase,
                          public IAeroModel,
                          public gnc::interfaces::IObservable {
public:
    Table2DAero()
        : ComponentBase("Table2DAero"),
          lift_table_({0.0, 1.0}, {0.0, 1.0}, {{0.0, 0.0}, {0.0, 0.0}}),
          drag_table_({0.0, 1.0}, {0.0, 1.0}, {{0.0, 0.0}, {0.0, 0.0}}) {}

    void configure(const gnc::core::ConfigNode& config) override {
        const auto source =
            gnc::vehicle::common::assets::loadConfiguredJsonAsset(config,
                                                                  "aero.table2d");
        reference_area_m2_ = source["reference_area_m2"].asDouble(reference_area_m2_);
        reference_length_m_ = source["reference_length_m"].asDouble(reference_length_m_);

        if (!source.has("alpha_breaks_rad") &&
            !source.has("mach_breaks") &&
            !source.has("lift_coefficients") &&
            !source.has("drag_coefficients")) {
            return;
        }

        const auto alpha_breaks_rad = readDoubleArray(source["alpha_breaks_rad"],
                                                      "alpha_breaks_rad");
        const auto mach_breaks = readDoubleArray(source["mach_breaks"], "mach_breaks");
        const auto lift_coefficients =
            readTable(source["lift_coefficients"],
                      alpha_breaks_rad.size(),
                      mach_breaks.size(),
                      "lift_coefficients");
        const auto drag_coefficients =
            readTable(source["drag_coefficients"],
                      alpha_breaks_rad.size(),
                      mach_breaks.size(),
                      "drag_coefficients");

        configureTables(alpha_breaks_rad,
                        mach_breaks,
                        lift_coefficients,
                        drag_coefficients);
    }

    void update(double) override {}

    AeroCoefficients computeCoefficients(double angle_of_attack_rad,
                                         double,
                                         double mach_number) const override {
        AeroCoefficients coefficients;
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
    static std::vector<double> readDoubleArray(const gnc::core::ConfigNode& node,
                                               const std::string& field_name) {
        if (!node.isArray() || node.size() < 2) {
            throw std::runtime_error("aero.table2d requires array field '" + field_name +
                                     "' with at least 2 entries.");
        }

        std::vector<double> values;
        values.reserve(node.size());
        for (size_t i = 0; i < node.size(); ++i) {
            values.push_back(node[i].asDouble());
        }
        return values;
    }

    static std::vector<std::vector<double>> readTable(const gnc::core::ConfigNode& node,
                                                      size_t expected_rows,
                                                      size_t expected_columns,
                                                      const std::string& field_name) {
        if (!node.isArray() || node.size() != expected_rows) {
            throw std::runtime_error("aero.table2d field '" + field_name +
                                     "' must provide exactly " +
                                     std::to_string(expected_rows) + " rows.");
        }

        std::vector<std::vector<double>> table;
        table.reserve(expected_rows);
        for (size_t row = 0; row < node.size(); ++row) {
            const auto& row_node = node[row];
            if (!row_node.isArray() || row_node.size() != expected_columns) {
                throw std::runtime_error("aero.table2d field '" + field_name +
                                         "' row " + std::to_string(row) +
                                         " must contain exactly " +
                                         std::to_string(expected_columns) + " values.");
            }

            std::vector<double> values;
            values.reserve(expected_columns);
            for (size_t column = 0; column < row_node.size(); ++column) {
                values.push_back(row_node[column].asDouble());
            }
            table.push_back(std::move(values));
        }
        return table;
    }

    void configureTables(const std::vector<double>& alpha_breaks_rad,
                         const std::vector<double>& mach_breaks,
                         const std::vector<std::vector<double>>& lift_coefficients,
                         const std::vector<std::vector<double>>& drag_coefficients) {
        lift_table_ = gnc::math::LookupTable2D(alpha_breaks_rad,
                                               mach_breaks,
                                               lift_coefficients);
        drag_table_ = gnc::math::LookupTable2D(alpha_breaks_rad,
                                               mach_breaks,
                                               drag_coefficients);
    }

    gnc::math::LookupTable2D lift_table_;
    gnc::math::LookupTable2D drag_table_;
    double reference_area_m2_ = 1.0;
    double reference_length_m_ = 1.0;
    mutable double last_angle_of_attack_rad_ = 0.0;
    mutable double last_mach_number_ = 0.0;
    mutable AeroCoefficients last_coefficients_{};
};

} // namespace gnc::vehicle::output
