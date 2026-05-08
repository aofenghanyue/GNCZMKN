#pragma once

#include "gnc/vehicle/process/interfaces/types_3dof.hpp"

namespace gnc::vehicle::process {

class IGuidance3Dof {
public:
    virtual ~IGuidance3Dof() = default;
    virtual const GuidanceCommand3Dof& guidanceCommand3Dof() const = 0;
};

} // namespace gnc::vehicle::process
