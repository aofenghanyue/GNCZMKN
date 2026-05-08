#pragma once

#include "gnc/common/math/eigen_types.hpp"

#include <string>

namespace gnc::vehicle::process {

struct PhaseState3Dof {
    std::string phase_name = "ideal";
    double elapsed_time_s = 0.0;
};

struct TrajectoryCommand3Dof {
    gnc::math::Vector3 desired_position_ecef_m = gnc::math::Vector3::Zero();
    double desired_altitude_m = 0.0;
    double desired_speed_mps = 0.0;
};

struct NavigationState3Dof {
    gnc::math::Vector3 position_ecef_m = gnc::math::Vector3::Zero();
    gnc::math::Vector3 velocity_ecef_mps = gnc::math::Vector3::Zero();
    double longitude_rad = 0.0;
    double latitude_rad = 0.0;
    double altitude_m = 0.0;
    double speed_mps = 0.0;
    double flight_path_angle_rad = 0.0;
    double heading_angle_rad = 0.0;
    double mach_number = 0.0;
};

struct TargetTrack3Dof {
    gnc::math::Vector3 relative_position_ecef_m = gnc::math::Vector3::Zero();
    gnc::math::Vector3 relative_velocity_ecef_mps = gnc::math::Vector3::Zero();
    gnc::math::Vector3 line_of_sight_ecef = gnc::math::Vector3::Zero();
    double range_m = 0.0;
    double closing_speed_mps = 0.0;
};

struct GuidanceCommand3Dof {
    gnc::math::Vector3 line_of_sight_ecef = gnc::math::Vector3::Zero();
    gnc::math::Vector3 acceleration_command_nue_mps2 = gnc::math::Vector3::Zero();
    double angle_of_attack_rad = 0.0;
    double bank_angle_rad = 0.0;
};

struct FlightControlCommand3Dof {
    double angle_of_attack_rad = 0.0;
    double bank_angle_rad = 0.0;
};

struct ControlAllocationCommand3Dof {
    double angle_of_attack_rad = 0.0;
    double bank_angle_rad = 0.0;
};

} // namespace gnc::vehicle::process
