#pragma once

#include "gnc/common/math/eigen_types.hpp"

namespace gnc::services::soviet_coord {

class IBodyAttitudeProvider {
public:
    virtual ~IBodyAttitudeProvider() = default;
    virtual gnc::math::Matrix3 getBodyToLaunchRotation() const = 0;
};

} // namespace gnc::services::soviet_coord
