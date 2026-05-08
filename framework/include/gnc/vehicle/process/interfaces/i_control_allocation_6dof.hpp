#pragma once

#include "gnc/vehicle/process/interfaces/types_6dof.hpp"

namespace gnc::vehicle::process {

class IControlAllocation6Dof {
public:
    virtual ~IControlAllocation6Dof() = default;
    virtual const ControlAllocationCommand6Dof& controlAllocationCommand6Dof()
        const = 0;
};

} // namespace gnc::vehicle::process
