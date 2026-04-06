#pragma once

#include "gnc/common/math_types.hpp"

namespace gnc::interfaces {

struct OverloadCommand {
    Vector3d overload_cmd;
    double timestamp = 0.0;
};

class IGuidanceOverload {
public:
    virtual ~IGuidanceOverload() = default;
    virtual const OverloadCommand& getOverloadCommand() const = 0;
    virtual bool isActive() const = 0;
};

} // namespace gnc::interfaces
