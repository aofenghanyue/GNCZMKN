#pragma once

#include "gnc/plugins/state_3dof/interfaces/flight_command_3dof.hpp"

namespace gnc::plugins::state_3dof {

class IFlightCommandProvider3DOF {
public:
    virtual ~IFlightCommandProvider3DOF() = default;

    virtual const FlightCommand3DOF& getFlightCommand() const = 0;
    virtual bool isActive() const = 0;
};

} // namespace gnc::plugins::state_3dof
