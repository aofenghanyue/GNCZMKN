#pragma once

#include "gnc/common/math/eigen_types.hpp"

namespace gnc::plugins::state_3dof {

class IStateSolver3DOF {
public:
    virtual ~IStateSolver3DOF() = default;

    virtual gnc::math::Vector3 getPosition() const = 0;
    virtual gnc::math::Vector3 getVelocity() const = 0;
    virtual double getMass() const = 0;
    virtual double getSpeed() const = 0;
    virtual double getAltitude() const = 0;
};

} // namespace gnc::plugins::state_3dof
