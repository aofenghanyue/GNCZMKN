#pragma once

#include "gnc/forms/local_spherical_3dof/types.hpp"

namespace gnc::forms::local_spherical_3dof {

class IFlightStateView {
public:
    virtual ~IFlightStateView() = default;

    virtual const FlightState& getFlightState() const = 0;
};

} // namespace gnc::forms::local_spherical_3dof
