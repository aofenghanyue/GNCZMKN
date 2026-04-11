#pragma once

namespace gnc::plugins::state_3dof {

struct FlightCommand3DOF {
    double angle_of_attack_rad = 0.0;
    double bank_angle_rad = 0.0;
    double timestamp = 0.0;
};

} // namespace gnc::plugins::state_3dof
