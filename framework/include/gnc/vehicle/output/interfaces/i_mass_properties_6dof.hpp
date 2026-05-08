#pragma once

#include "gnc/vehicle/interfaces/types_6dof.hpp"

namespace gnc::vehicle::output {

class IMassProperties6Dof {
public:
    virtual ~IMassProperties6Dof() = default;

    virtual gnc::vehicle::MassProperties6Dof massProperties6Dof() const = 0;
};

} // namespace gnc::vehicle::output
