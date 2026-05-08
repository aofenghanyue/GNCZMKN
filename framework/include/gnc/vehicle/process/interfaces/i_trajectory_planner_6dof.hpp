#pragma once

#include "gnc/vehicle/process/interfaces/types_6dof.hpp"

namespace gnc::vehicle::process {

class ITrajectoryPlanner6Dof {
public:
    virtual ~ITrajectoryPlanner6Dof() = default;
    virtual const TrajectoryPlan6Dof& trajectoryPlan6Dof() const = 0;
};

} // namespace gnc::vehicle::process
