#pragma once

#include "gnc/vehicle/process/interfaces/types_6dof.hpp"

namespace gnc::vehicle::process {

class IGuidance6Dof {
public:
    virtual ~IGuidance6Dof() = default;
    virtual const GuidanceCommand6Dof& guidanceCommand6Dof() const = 0;
};

} // namespace gnc::vehicle::process
