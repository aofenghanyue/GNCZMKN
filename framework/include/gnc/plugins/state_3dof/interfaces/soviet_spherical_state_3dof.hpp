#pragma once

namespace gnc::plugins::state_3dof {

struct SovietSphericalState3DOF {
    double longitude_rad = 0.0;
    double latitude_rad = 0.0;
    double altitude_m = 0.0;
    double speed_mps = 0.0;
    double flight_path_angle_rad = 0.0;
    double heading_angle_rad = 0.0;
};

} // namespace gnc::plugins::state_3dof
