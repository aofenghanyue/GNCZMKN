#pragma once

#include "gnc/common/math/eigen_types.hpp"

namespace gnc::services::soviet_coord {

class IBodyAirspeedProvider {
public:
    virtual ~IBodyAirspeedProvider() = default;
    virtual gnc::math::Vector3 getAirspeedInBodyFrame() const = 0;
};

} // namespace gnc::services::soviet_coord
