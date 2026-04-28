#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/common/assets/json_asset_loader.hpp"
#include "gnc/vehicle/output/interfaces/i_constant_mass.hpp"

#include <string>

namespace gnc::vehicle::output {

class ConstantMass final : public gnc::core::ComponentBase,
                           public IConstantMass,
                           public gnc::interfaces::IObservable {
public:
    ConstantMass() : ComponentBase("ConstantMass") {}

    void configure(const gnc::core::ConfigNode& config) override {
        configure(config, "config");
    }

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        const bool using_asset_file =
            gnc::vehicle::common::assets::hasConfiguredJsonAssetFile(config);
        const auto source =
            gnc::vehicle::common::assets::loadConfiguredJsonAsset(config,
                                                                  "mass.constant");
        const std::string source_path =
            using_asset_file
                ? "mass.constant asset '" +
                      gnc::vehicle::common::assets::resolveConfiguredJsonAssetPath(config)
                          .generic_string() +
                      "'"
                : config_path;
        gnc::core::ConfigReader reader(source, source_path);
        mass_kg_ = reader.requiredDouble("mass_kg");
        reader.validateNoUnknownKeys();
    }

    void update(double) override {}

    double getMassKg() const override { return mass_kg_; }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("mass_kg", [this]() { return mass_kg_; });
        return builder.build();
    }

private:
    double mass_kg_ = 1000.0;
};

} // namespace gnc::vehicle::output
