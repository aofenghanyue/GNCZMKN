#pragma once

#include "gnc/vehicle/process/interfaces/types_6dof.hpp"

namespace gnc::vehicle::process {

class IPhaseSequencer6Dof {
public:
    virtual ~IPhaseSequencer6Dof() = default;
    virtual const PhaseState6Dof& phaseState6Dof() const = 0;
};

} // namespace gnc::vehicle::process
