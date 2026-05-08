#pragma once

#include "gnc/common/math/eigen_types.hpp"

#include <Eigen/Geometry>

namespace gnc::forms::cartesian_6dof {

struct State {
    gnc::math::Vector3 position_m = gnc::math::Vector3::Zero();
    gnc::math::Vector3 velocity_mps = gnc::math::Vector3::Zero();
    Eigen::Quaterniond attitude_body_to_frame = Eigen::Quaterniond::Identity();
    gnc::math::Vector3 angular_rate_body_radps = gnc::math::Vector3::Zero();
};

struct Input {
    gnc::math::Vector3 acceleration_mps2 = gnc::math::Vector3::Zero();
    gnc::math::Vector3 angular_acceleration_body_radps2 =
        gnc::math::Vector3::Zero();
};

struct Truth {
    State state{};
    gnc::math::Vector3 acceleration_mps2 = gnc::math::Vector3::Zero();
    gnc::math::Vector3 angular_acceleration_body_radps2 =
        gnc::math::Vector3::Zero();
    double sample_time_s = 0.0;
};

struct StateDerivative {
    gnc::math::Vector3 position_rate_mps = gnc::math::Vector3::Zero();
    gnc::math::Vector3 velocity_rate_mps2 = gnc::math::Vector3::Zero();
    gnc::math::Vector4 attitude_rate_body_to_frame =
        gnc::math::Vector4::Zero();
    gnc::math::Vector3 angular_rate_body_radps2 = gnc::math::Vector3::Zero();
};

} // namespace gnc::forms::cartesian_6dof
