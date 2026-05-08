#pragma once

#include "gnc/vehicle/interfaces/types_6dof.hpp"

namespace gnc::vehicle::output {

class IPropulsion6Dof {
public:
    virtual ~IPropulsion6Dof() = default;

    virtual gnc::vehicle::ForceMoment6Dof propulsionForceMoment6Dof() const = 0;
};

} // namespace gnc::vehicle::output
