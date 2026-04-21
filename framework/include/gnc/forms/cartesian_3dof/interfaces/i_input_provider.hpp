#pragma once

#include "gnc/forms/cartesian_3dof/types.hpp"

namespace gnc::forms::cartesian_3dof {

class IInputProvider {
public:
    virtual ~IInputProvider() = default;

    virtual Input computeCartesian3DoFInput(const Truth& truth, double time_s) const = 0;
};

} // namespace gnc::forms::cartesian_3dof
