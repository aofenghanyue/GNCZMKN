#pragma once

#include "gnc/common/math_types.hpp"

namespace gnc::interfaces {

struct GuidanceCommand6DOF {
    Vector3d acceleration_cmd;
    Vector3d attitude_cmd;
    double timestamp = 0.0;
};

class IGuidance6DOF {
public:
    virtual ~IGuidance6DOF() = default;
    virtual const GuidanceCommand6DOF& getGuidanceCommand() const = 0;
    virtual void setTarget(const Vector3d& target_position) = 0;
    virtual bool isActive() const = 0;
};

} // namespace gnc::interfaces
