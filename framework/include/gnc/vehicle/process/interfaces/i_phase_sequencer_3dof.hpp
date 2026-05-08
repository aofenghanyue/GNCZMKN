#pragma once

#include "gnc/vehicle/process/interfaces/types_3dof.hpp"

namespace gnc::vehicle::process {

class IPhaseSequencer3Dof {
public:
    virtual ~IPhaseSequencer3Dof() = default;
    virtual const PhaseState3Dof& phaseState3Dof() const = 0;
};

} // namespace gnc::vehicle::process
