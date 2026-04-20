#pragma once

#include "gnc/common/math/eigen_types.hpp"
#include "gnc/plugins/state_3dof/interfaces/soviet_spherical_state_3dof.hpp"

namespace gnc::plugins::state_3dof {

class IAccelerationProvider3DOF {
public:
    virtual ~IAccelerationProvider3DOF() = default;

    virtual gnc::math::Vector3 computeLocalAccelerationNue(
        const SovietSphericalState3DOF& state,
        double time_s) const = 0;
};

} // namespace gnc::plugins::state_3dof
