#pragma once

#include "gnc/common/math/eigen_types.hpp"

#include <Eigen/Geometry>

namespace gnc::forms::local_spherical_6dof {

struct State {
    double longitude_rad = 0.0;
    double latitude_rad = 0.0;
    double altitude_m = 0.0;
    gnc::math::Vector3 velocity_nue_mps = gnc::math::Vector3::Zero();
    Eigen::Quaterniond attitude_body_to_nue = Eigen::Quaterniond::Identity();
    gnc::math::Vector3 angular_rate_body_radps = gnc::math::Vector3::Zero();
};

struct Input {
    gnc::math::Vector3 local_acceleration_nue_mps2 = gnc::math::Vector3::Zero();
    gnc::math::Vector3 angular_acceleration_body_radps2 =
        gnc::math::Vector3::Zero();
};

struct Truth {
    State state{};
    gnc::math::Vector3 position_ecef_m = gnc::math::Vector3::Zero();
    gnc::math::Vector3 velocity_ecef_mps = gnc::math::Vector3::Zero();
    gnc::math::Vector3 local_acceleration_nue_mps2 = gnc::math::Vector3::Zero();
    gnc::math::Vector3 angular_acceleration_body_radps2 =
        gnc::math::Vector3::Zero();
    double earth_radius_m = 6371000.0;
    double earth_rotation_rate_rad_per_s = 0.0;
    double sample_time_s = 0.0;
};

struct StateDerivative {
    double longitude_rate_radps = 0.0;
    double latitude_rate_radps = 0.0;
    double altitude_rate_mps = 0.0;
    gnc::math::Vector3 velocity_rate_nue_mps2 = gnc::math::Vector3::Zero();
    gnc::math::Vector4 attitude_rate_body_to_nue = gnc::math::Vector4::Zero();
    gnc::math::Vector3 angular_rate_body_radps2 = gnc::math::Vector3::Zero();
};

} // namespace gnc::forms::local_spherical_6dof
