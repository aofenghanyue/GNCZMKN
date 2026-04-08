#pragma once

#include "gnc/common/math_types.hpp"

namespace gnc::interfaces {

class IAngularVelocityProvider {
public:
    virtual ~IAngularVelocityProvider() = default;
    virtual Vector3d getAngularVelocity() const = 0;
};

} // namespace gnc::interfaces
