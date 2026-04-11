#pragma once

#include "gnc/common/math/eigen_types.hpp"

namespace gnc::plugins::environment {

class IEarth {
public:
    virtual ~IEarth() = default;

    virtual double getEquatorialRadius() const = 0;
    virtual double getFlattening() const = 0;
    virtual double getRotationRate() const = 0;
    virtual gnc::math::Vector3 geodeticToEcef(double latitude_rad,
                                              double longitude_rad,
                                              double altitude_m) const = 0;
};

} // namespace gnc::plugins::environment
