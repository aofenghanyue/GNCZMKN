#pragma once

#include "gnc/common/math_types.hpp"

namespace gnc::interfaces {

class IVelocityProvider {
public:
    virtual ~IVelocityProvider() = default;
    virtual Vector3d getVelocity() const = 0;
};

} // namespace gnc::interfaces
