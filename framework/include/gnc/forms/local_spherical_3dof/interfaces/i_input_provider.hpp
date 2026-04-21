#pragma once

#include "gnc/forms/local_spherical_3dof/types.hpp"

namespace gnc::forms::local_spherical_3dof {

class IInputProvider {
public:
    virtual ~IInputProvider() = default;

    virtual Input computeLocalSpherical3DoFInput(const Truth& truth,
                                                 double time_s) const = 0;
};

} // namespace gnc::forms::local_spherical_3dof
