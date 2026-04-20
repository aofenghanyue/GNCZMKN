#pragma once

#include "gnc/common/math/eigen_types.hpp"

namespace gnc::plugins::flight_state_3dof {

struct FlightState3DOFSovietSample {
    double longitude_rad = 0.0;
    double latitude_rad = 0.0;
    double altitude_m = 0.0;
    double speed_mps = 0.0;
    double flight_path_angle_rad = 0.0;
    double heading_angle_rad = 0.0;
    gnc::math::Vector3 local_velocity_nue_mps = gnc::math::Vector3::Zero();
    gnc::math::Vector3 local_acceleration_nue_mps2 = gnc::math::Vector3::Zero();
    double angle_of_attack_rad = 0.0;
    double bank_angle_rad = 0.0;
    double dynamic_pressure_pa = 0.0;
    double density_kg_per_m3 = 0.0;
    double pressure_pa = 0.0;
    double temperature_k = 0.0;
    double speed_of_sound_mps = 0.0;
    double mach_number = 0.0;
};

} // namespace gnc::plugins::flight_state_3dof
