#pragma once

#include "gnc/common/math/eigen_types.hpp"
#include "gnc/vehicle/interfaces/types_6dof.hpp"

#include <string>

namespace gnc::vehicle::process {

struct PhaseState6Dof {
    std::string phase_name = "idle";
    double elapsed_time_s = 0.0;
};

struct TrajectoryPlan6Dof {
    gnc::math::Vector3 desired_position_ecef_m = gnc::math::Vector3::Zero();
    gnc::math::Vector3 desired_velocity_ecef_mps = gnc::math::Vector3::Zero();
};

struct NavigationSolution6Dof {
    gnc::math::Vector3 position_ecef_m = gnc::math::Vector3::Zero();
    gnc::math::Vector3 velocity_ecef_mps = gnc::math::Vector3::Zero();
    double altitude_m = 0.0;
    double mach_number = 0.0;
};

struct TargetTrack6Dof {
    gnc::math::Vector3 relative_position_ecef_m = gnc::math::Vector3::Zero();
    gnc::math::Vector3 relative_velocity_ecef_mps = gnc::math::Vector3::Zero();
    gnc::math::Vector3 line_of_sight_ecef = gnc::math::Vector3::Zero();
    double range_m = 0.0;
    double closing_speed_mps = 0.0;
};

struct GuidanceCommand6Dof {
    gnc::math::Vector3 line_of_sight_ecef = gnc::math::Vector3::Zero();
    gnc::math::Vector3 acceleration_command_nue_mps2 =
        gnc::math::Vector3::Zero();
};

struct AttitudeControlCommand6Dof {
    gnc::math::Vector3 body_rate_command_radps = gnc::math::Vector3::Zero();
    gnc::math::Vector3 moment_command_body_nm = gnc::math::Vector3::Zero();
};

struct ControlAllocationCommand6Dof {
    gnc::vehicle::ControlSurfaceState6Dof control_surfaces{};
};

} // namespace gnc::vehicle::process
