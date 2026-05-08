#pragma once

#include "gnc/common/math/eigen_types.hpp"

namespace gnc::vehicle::input {

struct ImuMeasurement3Dof {
    gnc::math::Vector3 local_acceleration_nue_mps2 = gnc::math::Vector3::Zero();
    double speed_mps = 0.0;
    double flight_path_angle_rad = 0.0;
    double heading_angle_rad = 0.0;
};

class IImu3Dof {
public:
    virtual ~IImu3Dof() = default;
    virtual const ImuMeasurement3Dof& imuMeasurement3Dof() const = 0;
};

} // namespace gnc::vehicle::input
