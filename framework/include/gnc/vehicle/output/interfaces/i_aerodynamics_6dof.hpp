#pragma once

#include "gnc/vehicle/interfaces/types_6dof.hpp"

namespace gnc::vehicle::output {

class IAerodynamics6Dof {
public:
    virtual ~IAerodynamics6Dof() = default;

    virtual gnc::vehicle::ForceMoment6Dof aerodynamicForceMoment6Dof() const = 0;
};

} // namespace gnc::vehicle::output
