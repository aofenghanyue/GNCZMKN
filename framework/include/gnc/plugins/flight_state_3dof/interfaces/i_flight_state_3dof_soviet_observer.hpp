#pragma once

#include "gnc/plugins/flight_state_3dof/interfaces/flight_state_3dof_soviet_sample.hpp"

namespace gnc::plugins::flight_state_3dof {

class IFlightState3DOFSovietObserver {
public:
    virtual ~IFlightState3DOFSovietObserver() = default;

    virtual const FlightState3DOFSovietSample& getFlightState3DOFSoviet() const = 0;
};

} // namespace gnc::plugins::flight_state_3dof
