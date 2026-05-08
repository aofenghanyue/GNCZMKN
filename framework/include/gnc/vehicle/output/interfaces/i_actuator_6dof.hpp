#pragma once

#include "gnc/vehicle/interfaces/types_6dof.hpp"

namespace gnc::vehicle::output {

class IActuator6Dof {
public:
    virtual ~IActuator6Dof() = default;

    virtual gnc::vehicle::ControlSurfaceState6Dof actuatorState6Dof() const = 0;
};

} // namespace gnc::vehicle::output
