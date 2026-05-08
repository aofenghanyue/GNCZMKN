#pragma once

namespace gnc::vehicle::output {

struct ActuatorState3Dof {
    double angle_of_attack_rad = 0.0;
    double bank_angle_rad = 0.0;
};

class IActuator3Dof {
public:
    virtual ~IActuator3Dof() = default;
    virtual const ActuatorState3Dof& actuatorState3Dof() const = 0;
};

} // namespace gnc::vehicle::output
