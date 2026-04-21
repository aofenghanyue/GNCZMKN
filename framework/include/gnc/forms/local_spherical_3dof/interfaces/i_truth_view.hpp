#pragma once

#include "gnc/forms/local_spherical_3dof/types.hpp"

namespace gnc::forms::local_spherical_3dof {

class ITruthView {
public:
    virtual ~ITruthView() = default;

    virtual const Truth& getLocalSpherical3DoFTruth() const = 0;
};

} // namespace gnc::forms::local_spherical_3dof
