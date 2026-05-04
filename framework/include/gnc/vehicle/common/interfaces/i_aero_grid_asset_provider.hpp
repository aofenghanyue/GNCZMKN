#pragma once

#include "gnc/vehicle/common/assets/aero_grid_asset.hpp"

namespace gnc::vehicle::common {

class IAeroGridAssetProvider {
public:
    virtual ~IAeroGridAssetProvider() = default;

    virtual const gnc::vehicle::common::assets::AeroGridAsset&
    getAeroGridAsset() const = 0;
};

} // namespace gnc::vehicle::common
