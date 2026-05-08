#pragma once

#include "gnc/common/math/eigen_types.hpp"

namespace gnc::vehicle::input {

struct ImuMeasurement6Dof {
    gnc::math::Vector3 local_acceleration_nue_mps2 = gnc::math::Vector3::Zero();
    gnc::math::Vector3 angular_rate_body_radps = gnc::math::Vector3::Zero();
    gnc::math::Vector3 angular_acceleration_body_radps2 =
        gnc::math::Vector3::Zero();
};

class IImu6Dof {
public:
    virtual ~IImu6Dof() = default;

    virtual const ImuMeasurement6Dof& imuMeasurement6Dof() const = 0;
};

} // namespace gnc::vehicle::input
