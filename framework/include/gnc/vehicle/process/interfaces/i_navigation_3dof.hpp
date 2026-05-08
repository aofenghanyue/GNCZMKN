#pragma once

#include "gnc/vehicle/process/interfaces/types_3dof.hpp"

namespace gnc::vehicle::process {

class INavigation3Dof {
public:
    virtual ~INavigation3Dof() = default;
    virtual const NavigationState3Dof& navigationState3Dof() const = 0;
};

} // namespace gnc::vehicle::process
