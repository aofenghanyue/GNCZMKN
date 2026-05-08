#pragma once

#include "gnc/vehicle/process/interfaces/types_6dof.hpp"

namespace gnc::vehicle::process {

class ITargetTracking6Dof {
public:
    virtual ~ITargetTracking6Dof() = default;
    virtual const TargetTrack6Dof& targetTrack6Dof() const = 0;
};

} // namespace gnc::vehicle::process
