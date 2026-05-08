#pragma once

#include "gnc/vehicle/process/interfaces/types_3dof.hpp"

namespace gnc::vehicle::process {

class IControlAllocation3Dof {
public:
    virtual ~IControlAllocation3Dof() = default;
    virtual const ControlAllocationCommand3Dof& controlAllocationCommand3Dof()
        const = 0;
};

} // namespace gnc::vehicle::process
