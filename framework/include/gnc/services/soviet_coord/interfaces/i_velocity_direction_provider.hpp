#pragma once

#include "gnc/common/math/eigen_types.hpp"

namespace gnc::services::soviet_coord {

class IVelocityDirectionProvider {
public:
    virtual ~IVelocityDirectionProvider() = default;
    virtual gnc::math::Vector3 getVelocityInLaunchFrame() const = 0;
};

} // namespace gnc::services::soviet_coord
