#pragma once

#include "gnc/plugins/state_3dof/interfaces/soviet_spherical_state_3dof.hpp"

namespace gnc::plugins::state_3dof {

class ISovietSphericalState3DOF {
public:
    virtual ~ISovietSphericalState3DOF() = default;

    virtual SovietSphericalState3DOF getSovietSphericalState() const = 0;
};

} // namespace gnc::plugins::state_3dof
