#pragma once

#include "gnc/common/math/eigen_types.hpp"

#include <Eigen/Geometry>

namespace gnc::vehicle::input {

struct SatelliteNavMeasurement6Dof {
    gnc::math::Vector3 position_ecef_m = gnc::math::Vector3::Zero();
    gnc::math::Vector3 velocity_ecef_mps = gnc::math::Vector3::Zero();
    Eigen::Quaterniond attitude_body_to_nue = Eigen::Quaterniond::Identity();
    double longitude_rad = 0.0;
    double latitude_rad = 0.0;
    double altitude_m = 0.0;
};

class ISatelliteNav6Dof {
public:
    virtual ~ISatelliteNav6Dof() = default;

    virtual const SatelliteNavMeasurement6Dof& satelliteNavMeasurement6Dof()
        const = 0;
};

} // namespace gnc::vehicle::input
