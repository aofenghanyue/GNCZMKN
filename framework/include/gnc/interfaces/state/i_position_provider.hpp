#pragma once

#include "gnc/common/math_types.hpp"

namespace gnc::interfaces {

class IPositionProvider {
public:
    virtual ~IPositionProvider() = default;
    virtual Vector3d getPosition() const = 0;
};

} // namespace gnc::interfaces
