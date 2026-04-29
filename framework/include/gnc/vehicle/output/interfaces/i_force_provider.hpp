#pragma once

#include "gnc/common/math/eigen_types.hpp"

namespace gnc::vehicle::output {

class IForceProvider {
public:
    virtual ~IForceProvider() = default;

    virtual gnc::math::Vector3 getForceN() const = 0;
};

} // namespace gnc::vehicle::output
