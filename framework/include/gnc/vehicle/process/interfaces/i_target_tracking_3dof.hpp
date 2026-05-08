#pragma once

#include "gnc/vehicle/process/interfaces/types_3dof.hpp"

namespace gnc::vehicle::process {

class ITargetTracking3Dof {
public:
    virtual ~ITargetTracking3Dof() = default;
    virtual const TargetTrack3Dof& targetTrack3Dof() const = 0;
};

} // namespace gnc::vehicle::process
