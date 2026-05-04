#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/vehicle/common/assets/aero_grid_asset.hpp"
#include "gnc/vehicle/common/assets/json_asset_loader.hpp"
#include "gnc/vehicle/common/interfaces/i_aero_grid_asset_provider.hpp"

#include <string>

namespace gnc::vehicle::common {

class AeroGridAssetProvider final
    : public gnc::core::ComponentBase,
      public IAeroGridAssetProvider {
public:
    AeroGridAssetProvider() : ComponentBase("AeroGridAssetProvider") {}

    void configure(const gnc::core::ConfigNode& config) override {
        configure(config, "config");
    }

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        const auto source =
            gnc::vehicle::common::assets::loadConfiguredJsonAsset(config,
                                                                  "aero.asset.grid",
                                                                  config_path);
        asset_ = gnc::vehicle::common::assets::parseAeroGridAsset(
            source,
            gnc::vehicle::common::assets::hasConfiguredJsonAssetFile(config,
                                                                     config_path)
                ? "aero.asset.grid asset '" +
                      gnc::vehicle::common::assets::resolveConfiguredJsonAssetPath(
                          config,
                          config_path)
                          .generic_string() +
                      "'"
                : config_path);
    }

    void update(double) override {}

    const gnc::vehicle::common::assets::AeroGridAsset&
    getAeroGridAsset() const override {
        return asset_;
    }

private:
    gnc::vehicle::common::assets::AeroGridAsset asset_;
};

} // namespace gnc::vehicle::common
