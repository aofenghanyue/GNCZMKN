#pragma once

#include "gnc/forms/target_point/types.hpp"

namespace gnc::forms::target_point {

class ITruthView {
public:
    virtual ~ITruthView() = default;

    virtual const Truth& getTargetPointTruth() const = 0;
};

} // namespace gnc::forms::target_point
