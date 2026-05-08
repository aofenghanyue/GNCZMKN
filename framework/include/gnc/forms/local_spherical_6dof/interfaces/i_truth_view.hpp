#pragma once

#include "gnc/forms/local_spherical_6dof/types.hpp"

namespace gnc::forms::local_spherical_6dof {

class ITruthView {
public:
    virtual ~ITruthView() = default;

    virtual const Truth& getLocalSpherical6DoFTruth() const = 0;
};

} // namespace gnc::forms::local_spherical_6dof
