#pragma once

#include "gnc/common/math/eigen_types.hpp"

namespace gnc::vehicle::input {

struct SeekerMeasurement3Dof {
    gnc::math::Vector3 relative_position_ecef_m = gnc::math::Vector3::Zero();
    gnc::math::Vector3 relative_velocity_ecef_mps = gnc::math::Vector3::Zero();
    gnc::math::Vector3 line_of_sight_ecef = gnc::math::Vector3::Zero();
    double range_m = 0.0;
    double closing_speed_mps = 0.0;
};

class ISeeker3Dof {
public:
    virtual ~ISeeker3Dof() = default;
    virtual const SeekerMeasurement3Dof& seekerMeasurement3Dof() const = 0;
};

} // namespace gnc::vehicle::input
