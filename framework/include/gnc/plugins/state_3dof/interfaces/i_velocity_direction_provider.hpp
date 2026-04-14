#pragma once

#include "gnc/common/math/eigen_types.hpp"

namespace gnc::plugins::state_3dof {

class IVelocityDirectionProvider {
public:
    virtual ~IVelocityDirectionProvider() = default;
    virtual gnc::math::Vector3 getVelocityInLaunchFrame() const = 0;
};

} // namespace gnc::plugins::state_3dof
