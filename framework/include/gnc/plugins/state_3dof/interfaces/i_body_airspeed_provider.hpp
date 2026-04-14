#pragma once

#include "gnc/common/math/eigen_types.hpp"

namespace gnc::plugins::state_3dof {

class IBodyAirspeedProvider {
public:
    virtual ~IBodyAirspeedProvider() = default;
    virtual gnc::math::Vector3 getAirspeedInBodyFrame() const = 0;
};

} // namespace gnc::plugins::state_3dof
