#pragma once

#include "gnc/common/math/eigen_types.hpp"

namespace gnc::vehicle::input {

struct SatelliteNavMeasurement3Dof {
    gnc::math::Vector3 position_ecef_m = gnc::math::Vector3::Zero();
    gnc::math::Vector3 velocity_ecef_mps = gnc::math::Vector3::Zero();
    double longitude_rad = 0.0;
    double latitude_rad = 0.0;
    double altitude_m = 0.0;
    double speed_mps = 0.0;
    double flight_path_angle_rad = 0.0;
    double heading_angle_rad = 0.0;
};

class ISatelliteNav3Dof {
public:
    virtual ~ISatelliteNav3Dof() = default;
    virtual const SatelliteNavMeasurement3Dof& satelliteNavMeasurement3Dof()
        const = 0;
};

} // namespace gnc::vehicle::input
