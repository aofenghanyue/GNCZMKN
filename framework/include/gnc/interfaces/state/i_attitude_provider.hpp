#pragma once

#include "gnc/common/math_types.hpp"

namespace gnc::interfaces {

class IAttitudeProvider {
public:
    virtual ~IAttitudeProvider() = default;
    virtual Quaterniond getAttitude() const = 0;
};

} // namespace gnc::interfaces
