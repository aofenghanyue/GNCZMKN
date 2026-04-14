#pragma once

#include "gnc/common/math/eigen_types.hpp"

namespace gnc::plugins::state_3dof {

class IBodyAttitudeProvider {
public:
    virtual ~IBodyAttitudeProvider() = default;
    virtual gnc::math::Matrix3 getBodyToLaunchRotation() const = 0;
};

} // namespace gnc::plugins::state_3dof
