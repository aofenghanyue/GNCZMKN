#pragma once

#include "gnc/common/math/eigen_types.hpp"

namespace gnc::forms::target_point {

struct State {
    gnc::math::Vector3 position_ecef_m = gnc::math::Vector3::Zero();
    gnc::math::Vector3 velocity_ecef_mps = gnc::math::Vector3::Zero();
};

struct Truth {
    State state{};
    gnc::math::Vector3 position_ecef_m = gnc::math::Vector3::Zero();
    gnc::math::Vector3 velocity_ecef_mps = gnc::math::Vector3::Zero();
    double sample_time_s = 0.0;
};

} // namespace gnc::forms::target_point
