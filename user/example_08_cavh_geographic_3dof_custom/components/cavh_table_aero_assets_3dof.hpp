#pragma once

#include "cavh_table2d.hpp"
#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/common/interfaces/i_aerodynamic_assets_3dof.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <stdexcept>

namespace cavh::components {

class TableAeroAssets3Dof final
    : public gnc::core::ComponentBase,
      public gnc::vehicle::common::IAerodynamicAssets3Dof,
      public gnc::interfaces::IObservable {
public:
    TableAeroAssets3Dof() : ComponentBase("CavhTableAeroAssets3Dof") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        asset_file_ = reader.requiredString("asset_file");
        reader.validateNoUnknownKeys();
    }

    void initialize() override { ensureLoaded(); }

    void update(double) override {}

    gnc::vehicle::output::AeroCoefficients sampleAeroCoefficients3Dof(
        const gnc::vehicle::common::AeroQuery3Dof& query) const override {
        ensureLoaded();
        last_query_ = query;
        const auto sample =
            table_->sample(query.angle_of_attack_rad, query.mach_number);
        last_coefficients_ = {};
        last_coefficients_.lift_coefficient = sample.lift_coefficient;
        last_coefficients_.drag_coefficient = sample.drag_coefficient;
        return last_coefficients_;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields()
        const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("angle_of_attack_rad",
                          [this]() { return last_query_.angle_of_attack_rad; });
        builder.addScalar("mach_number",
                          [this]() { return last_query_.mach_number; });
        builder.addScalar("lift_coefficient",
                          [this]() { return last_coefficients_.lift_coefficient; });
        builder.addScalar("drag_coefficient",
                          [this]() { return last_coefficients_.drag_coefficient; });
        return builder.build();
    }

private:
    void ensureLoaded() const {
        if (!table_) {
            table_.emplace(CavhAeroTable2D::loadFromCsv(asset_file_));
        }
    }

    std::string asset_file_;
    mutable std::optional<CavhAeroTable2D> table_;
    mutable gnc::vehicle::common::AeroQuery3Dof last_query_{};
    mutable gnc::vehicle::output::AeroCoefficients last_coefficients_{};
};

} // namespace cavh::components

GNC_REGISTER_COMPONENT_TYPE(
    "cavh.common.aero_assets_3dof.table2d",
    cavh::components::TableAeroAssets3Dof,
    ::gnc::core::ComponentPackageRole::VehicleCommon,
    ::gnc::core::ExecutionStage::None,
    "local_spherical_3dof",
    ::gnc::vehicle::common::IAerodynamicAssets3Dof,
    ::gnc::interfaces::IObservable)
