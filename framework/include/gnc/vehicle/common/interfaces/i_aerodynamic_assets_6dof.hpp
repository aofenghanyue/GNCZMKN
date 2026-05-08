#pragma once

#include "gnc/vehicle/interfaces/types_6dof.hpp"

namespace gnc::vehicle::common {

class IAerodynamicAssets6Dof {
public:
    virtual ~IAerodynamicAssets6Dof() = default;

    virtual gnc::vehicle::AeroCoefficients6Dof sampleAeroCoefficients6Dof(
        const gnc::vehicle::AeroQuery6Dof& query) const = 0;
};

} // namespace gnc::vehicle::common
