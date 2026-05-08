#pragma once

#include "gnc/vehicle/process/interfaces/types_6dof.hpp"

namespace gnc::vehicle::process {

class INavigation6Dof {
public:
    virtual ~INavigation6Dof() = default;
    virtual const NavigationSolution6Dof& navigationSolution6Dof() const = 0;
};

} // namespace gnc::vehicle::process
