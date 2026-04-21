#pragma once

#include "gnc/common/math/eigen_types.hpp"

namespace gnc::forms::local_spherical_3dof {

struct State {
    double longitude_rad = 0.0;
    double latitude_rad = 0.0;
    double altitude_m = 0.0;
    double speed_mps = 0.0;
    double flight_path_angle_rad = 0.0;
    double heading_angle_rad = 0.0;
};

struct Input {
    gnc::math::Vector3 local_acceleration_nue_mps2 = gnc::math::Vector3::Zero();
};

struct Truth {
    State state{};
    gnc::math::Vector3 position_ecef_m = gnc::math::Vector3::Zero();
    gnc::math::Vector3 velocity_ecef_mps = gnc::math::Vector3::Zero();
    gnc::math::Vector3 local_velocity_nue_mps = gnc::math::Vector3::Zero();
    gnc::math::Vector3 local_acceleration_nue_mps2 = gnc::math::Vector3::Zero();
    gnc::math::Vector3 drag_axis_nue = gnc::math::Vector3::Zero();
    gnc::math::Vector3 lift_up_axis_nue = gnc::math::Vector3::Zero();
    gnc::math::Vector3 lift_side_axis_nue = gnc::math::Vector3::Zero();
    gnc::math::Vector3 velocity_launch_mps = gnc::math::Vector3::Zero();
    double launch_azimuth_rad = 0.0;
    double earth_radius_m = 0.0;
    double earth_rotation_rate_rad_per_s = 0.0;
    double sample_time_s = 0.0;
};

struct FlightState {
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

struct StateDerivative {
    double longitude_rate_rad_per_s = 0.0;
    double latitude_rate_rad_per_s = 0.0;
    double altitude_rate_m_per_s = 0.0;
    double speed_rate_m_per_s2 = 0.0;
    double flight_path_angle_rate_rad_per_s = 0.0;
    double heading_angle_rate_rad_per_s = 0.0;
};

} // namespace gnc::forms::local_spherical_3dof
