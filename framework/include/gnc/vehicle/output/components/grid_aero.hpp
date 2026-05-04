#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/common/assets/aero_grid_asset.hpp"
#include "gnc/vehicle/common/assets/json_asset_loader.hpp"
#include "gnc/vehicle/common/interfaces/i_aero_grid_asset_provider.hpp"
#include "gnc/vehicle/output/interfaces/i_aero_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace gnc::vehicle::output {

class GridAero final : public gnc::core::ComponentBase,
                       public IAeroModel,
                       public gnc::interfaces::IObservable {
public:
    GridAero() : ComponentBase("GridAero") {}

    void configure(const gnc::core::ConfigNode& config) override {
        configure(config, "config");
    }

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        asset_lookup_name_ = reader.optionalString("asset", "aero_asset");

        if (gnc::vehicle::common::assets::hasConfiguredJsonAssetFile(config,
                                                                     config_path)) {
            const auto source =
                gnc::vehicle::common::assets::loadConfiguredJsonAsset(config,
                                                                      "aero.grid",
                                                                      config_path);
            asset_ = gnc::vehicle::common::assets::parseAeroGridAsset(
                source,
                "aero.grid asset '" +
                    gnc::vehicle::common::assets::resolveConfiguredJsonAssetPath(
                        config,
                        config_path)
                        .generic_string() +
                    "'");
            direct_asset_loaded_ = true;
            asset_ready_ = true;
        }

        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        if (direct_asset_loaded_) {
            return;
        }
        registry.bindAll(gnc::core::bind(asset_provider_, asset_lookup_name_));
        asset_ = asset_provider_->getAeroGridAsset();
        asset_ready_ = true;
    }

    void update(double) override {}

    AeroCoefficients computeCoefficients(double angle_of_attack_rad,
                                         double sideslip_angle_rad,
                                         double mach_number) const override {
        if (!asset_ready_) {
            throw std::runtime_error("aero.grid has no configured aero grid asset.");
        }

        const QueryPoint query{
            angle_of_attack_rad,
            sideslip_angle_rad,
            mach_number,
        };

        AeroCoefficients coefficients;
        if (asset_.hasColumn("c_lift") && asset_.hasColumn("c_drag")) {
            coefficients.lift_coefficient = interpolateColumn("c_lift", query);
            coefficients.drag_coefficient = interpolateColumn("c_drag", query);
        } else if (asset_.hasColumn("c_axial") && asset_.hasColumn("c_normal")) {
            const double axial = interpolateColumn("c_axial", query);
            const double normal = interpolateColumn("c_normal", query);
            coefficients.drag_coefficient =
                axial * std::cos(angle_of_attack_rad) +
                normal * std::sin(angle_of_attack_rad);
            coefficients.lift_coefficient =
                normal * std::cos(angle_of_attack_rad) -
                axial * std::sin(angle_of_attack_rad);
        } else {
            throw std::runtime_error(
                "aero.grid asset must provide either c_lift/c_drag or c_axial/c_normal.");
        }

        if (asset_.hasColumn("c_side")) {
            coefficients.side_force_coefficient = interpolateColumn("c_side", query);
        }
        if (asset_.hasColumn("c_roll")) {
            coefficients.rolling_moment_coefficient = interpolateColumn("c_roll", query);
        }
        if (asset_.hasColumn("c_pitch")) {
            coefficients.pitching_moment_coefficient = interpolateColumn("c_pitch", query);
        }
        if (asset_.hasColumn("c_yaw")) {
            coefficients.yawing_moment_coefficient = interpolateColumn("c_yaw", query);
        }

        last_angle_of_attack_rad_ = angle_of_attack_rad;
        last_sideslip_angle_rad_ = sideslip_angle_rad;
        last_mach_number_ = mach_number;
        last_coefficients_ = coefficients;
        return coefficients;
    }

    double getReferenceArea() const override {
        return asset_ready_ ? asset_.reference.area_m2 : 1.0;
    }

    double getReferenceLength() const override {
        return asset_ready_ ? asset_.reference.length_m : 1.0;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("angle_of_attack_rad",
                          [this]() { return last_angle_of_attack_rad_; });
        builder.addScalar("sideslip_angle_rad",
                          [this]() { return last_sideslip_angle_rad_; });
        builder.addScalar("mach_number", [this]() { return last_mach_number_; });
        builder.addScalar("lift_coefficient",
                          [this]() { return last_coefficients_.lift_coefficient; });
        builder.addScalar("drag_coefficient",
                          [this]() { return last_coefficients_.drag_coefficient; });
        builder.addScalar("side_force_coefficient",
                          [this]() {
                              return last_coefficients_.side_force_coefficient;
                          });
        builder.addScalar("pitching_moment_coefficient",
                          [this]() {
                              return last_coefficients_.pitching_moment_coefficient;
                          });
        return builder.build();
    }

private:
    struct QueryPoint {
        double alpha_rad = 0.0;
        double beta_rad = 0.0;
        double mach = 0.0;
    };

    struct Bracket {
        size_t lower = 0;
        double t = 0.0;
    };

    static Bracket bracketAxis(const std::vector<double>& nodes, double value) {
        if (value <= nodes.front()) {
            return {0, 0.0};
        }
        if (value >= nodes.back()) {
            return {nodes.size() - 2, 1.0};
        }
        auto upper = std::upper_bound(nodes.begin(), nodes.end(), value);
        const size_t upper_index = static_cast<size_t>(upper - nodes.begin());
        const size_t lower_index = upper_index - 1;
        const double lo = nodes[lower_index];
        const double hi = nodes[upper_index];
        return {lower_index, (value - lo) / (hi - lo)};
    }

    static double axisValue(const std::string& axis_name, const QueryPoint& query) {
        if (axis_name == "alpha_rad") {
            return query.alpha_rad;
        }
        if (axis_name == "beta_rad") {
            return query.beta_rad;
        }
        if (axis_name == "mach") {
            return query.mach;
        }
        throw std::runtime_error("aero.grid cannot query unsupported axis '" +
                                 axis_name + "'.");
    }

    const gnc::vehicle::common::assets::AeroGridAxis& axisByName(
        const std::string& name) const {
        const auto it = std::find_if(
            asset_.axes.begin(),
            asset_.axes.end(),
            [&](const gnc::vehicle::common::assets::AeroGridAxis& axis) {
                return axis.name == name;
            });
        if (it == asset_.axes.end()) {
            throw std::runtime_error("aero.grid layout references unknown axis '" +
                                     name + "'.");
        }
        return *it;
    }

    std::vector<size_t> computeStrides() const {
        const auto& order = asset_.layout.axis_order;
        if (order.empty()) {
            throw std::runtime_error("aero.grid layout has no axis order.");
        }
        if (asset_.layout.fastest_varying_axis != order.back()) {
            throw std::runtime_error(
                "aero.grid first implementation requires fastest_varying_axis "
                "to equal the last axis_order entry.");
        }

        std::vector<size_t> strides(order.size(), 1);
        for (size_t reverse = order.size() - 1; reverse > 0; --reverse) {
            const size_t previous = reverse - 1;
            strides[previous] =
                strides[reverse] * axisByName(order[reverse]).nodes.size();
        }
        return strides;
    }

    double interpolateColumn(const std::string& column_name,
                             const QueryPoint& query) const {
        const size_t column = asset_.columnIndex(column_name);
        const auto& order = asset_.layout.axis_order;
        if (order.size() > 20) {
            throw std::runtime_error("aero.grid supports at most 20 axes.");
        }

        std::vector<Bracket> brackets;
        brackets.reserve(order.size());
        for (const auto& axis_name : order) {
            const auto& axis = axisByName(axis_name);
            brackets.push_back(bracketAxis(axis.nodes, axisValue(axis_name, query)));
        }

        const std::vector<size_t> strides = computeStrides();
        const size_t corner_count = size_t{1} << order.size();
        double value = 0.0;
        for (size_t corner = 0; corner < corner_count; ++corner) {
            size_t row = 0;
            double weight = 1.0;
            for (size_t axis_index = 0; axis_index < order.size(); ++axis_index) {
                const bool upper = ((corner >> axis_index) & size_t{1}) != 0;
                const auto& bracket = brackets[axis_index];
                row += (bracket.lower + (upper ? 1 : 0)) * strides[axis_index];
                weight *= upper ? bracket.t : (1.0 - bracket.t);
            }
            value += weight * asset_.data[row][column];
        }
        return value;
    }

    std::string asset_lookup_name_ = "aero_asset";
    gnc::vehicle::common::IAeroGridAssetProvider* asset_provider_ = nullptr;
    gnc::vehicle::common::assets::AeroGridAsset asset_;
    bool direct_asset_loaded_ = false;
    bool asset_ready_ = false;
    mutable double last_angle_of_attack_rad_ = 0.0;
    mutable double last_sideslip_angle_rad_ = 0.0;
    mutable double last_mach_number_ = 0.0;
    mutable AeroCoefficients last_coefficients_{};
};

} // namespace gnc::vehicle::output
