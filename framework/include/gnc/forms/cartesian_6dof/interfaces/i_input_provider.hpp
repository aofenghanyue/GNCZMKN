#pragma once

#include "gnc/forms/cartesian_6dof/types.hpp"

namespace gnc::forms::cartesian_6dof {

class IInputProvider {
public:
    virtual ~IInputProvider() = default;

    virtual Input computeCartesian6DoFInput(const Truth& truth,
                                            double time_s) const = 0;
};

} // namespace gnc::forms::cartesian_6dof
