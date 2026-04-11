#pragma once

#include "gnc/common/math/eigen_types.hpp"

namespace gnc::plugins::soviet_coord {

class IBodyAttitudeProvider {
public:
    virtual ~IBodyAttitudeProvider() = default;
    virtual gnc::math::Matrix3 getBodyToLaunchRotation() const = 0;
};

class IVelocityDirectionProvider {
public:
    virtual ~IVelocityDirectionProvider() = default;
    virtual gnc::math::Vector3 getVelocityInLaunchFrame() const = 0;
};

class IBodyAirspeedProvider {
public:
    virtual ~IBodyAirspeedProvider() = default;
    virtual gnc::math::Vector3 getAirspeedInBodyFrame() const = 0;
};

} // namespace gnc::plugins::soviet_coord
