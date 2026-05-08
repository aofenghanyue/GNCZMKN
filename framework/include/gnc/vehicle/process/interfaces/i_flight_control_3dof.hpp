#pragma once

#include "gnc/vehicle/process/interfaces/types_3dof.hpp"

namespace gnc::vehicle::process {

class IFlightControl3Dof {
public:
    virtual ~IFlightControl3Dof() = default;
    virtual const FlightControlCommand3Dof& flightControlCommand3Dof() const = 0;
};

} // namespace gnc::vehicle::process
