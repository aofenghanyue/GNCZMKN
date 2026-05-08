#pragma once

#include "gnc/vehicle/process/interfaces/types_6dof.hpp"

namespace gnc::vehicle::process {

class IAttitudeControl6Dof {
public:
    virtual ~IAttitudeControl6Dof() = default;
    virtual const AttitudeControlCommand6Dof& attitudeControlCommand6Dof()
        const = 0;
};

} // namespace gnc::vehicle::process
