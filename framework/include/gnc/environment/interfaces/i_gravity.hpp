#pragma once

#include "gnc/common/math/eigen_types.hpp"

namespace gnc::environment {

class IGravity {
public:
    virtual ~IGravity() = default;

    virtual double getSeaLevelGravity() const = 0;
    virtual double getGravityMagnitude(double altitude_m) const = 0;
    virtual gnc::math::Vector3 getGravityVector(
        const gnc::math::Vector3& position_ecef) const = 0;
};

} // namespace gnc::environment
