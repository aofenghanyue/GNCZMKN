#pragma once

#include "gnc/forms/cartesian_3dof/types.hpp"

namespace gnc::forms::cartesian_3dof {

class ITruthView {
public:
    virtual ~ITruthView() = default;

    virtual const Truth& getCartesian3DoFTruth() const = 0;
};

} // namespace gnc::forms::cartesian_3dof
