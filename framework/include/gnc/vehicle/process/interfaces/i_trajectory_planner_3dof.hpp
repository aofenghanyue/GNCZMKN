#pragma once

#include "gnc/vehicle/process/interfaces/types_3dof.hpp"

namespace gnc::vehicle::process {

class ITrajectoryPlanner3Dof {
public:
    virtual ~ITrajectoryPlanner3Dof() = default;
    virtual const TrajectoryCommand3Dof& trajectoryCommand3Dof() const = 0;
};

} // namespace gnc::vehicle::process
