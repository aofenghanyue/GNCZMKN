/**
 * @file rotations.hpp
 * @brief Low-level rotation helpers that stay independent of any concrete coordinate-system convention.
 */
#pragma once

#include "gnc/common/math/math.hpp"

namespace gnc::coord {

using gnc::math::Matrix3;
using gnc::math::Quaternion;
using gnc::math::Vector3;

inline Matrix3 rotationAboutX(double angle_rad) {
    return gnc::math::rotX(angle_rad);
}

inline Matrix3 rotationAboutY(double angle_rad) {
    return gnc::math::rotY(angle_rad);
}

inline Matrix3 rotationAboutZ(double angle_rad) {
    return gnc::math::rotZ(angle_rad);
}

} // namespace gnc::coord
