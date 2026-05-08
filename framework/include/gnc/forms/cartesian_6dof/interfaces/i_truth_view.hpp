#pragma once

#include "gnc/forms/cartesian_6dof/types.hpp"

namespace gnc::forms::cartesian_6dof {

class ITruthView {
public:
    virtual ~ITruthView() = default;

    virtual const Truth& getCartesian6DoFTruth() const = 0;
};

} // namespace gnc::forms::cartesian_6dof
