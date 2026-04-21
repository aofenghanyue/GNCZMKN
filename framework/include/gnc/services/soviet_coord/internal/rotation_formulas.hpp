#pragma once

#include "gnc/common/math/eigen_types.hpp"

#include <cmath>
#include <stdexcept>

namespace gnc::services::soviet_coord::internal {

inline gnc::math::Matrix3 rotationY(double angle_rad) {
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    gnc::math::Matrix3 rotation = gnc::math::Matrix3::Identity();
    rotation(0, 0) = c;
    rotation(0, 2) = s;
    rotation(2, 0) = -s;
    rotation(2, 2) = c;
    return rotation;
}

inline gnc::math::Matrix3 rotationZ(double angle_rad) {
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    gnc::math::Matrix3 rotation = gnc::math::Matrix3::Identity();
    rotation(0, 0) = c;
    rotation(0, 1) = -s;
    rotation(1, 0) = s;
    rotation(1, 1) = c;
    return rotation;
}

inline gnc::math::Matrix3 basisToParent(const gnc::math::Vector3& axis_x,
                                        const gnc::math::Vector3& axis_y,
                                        const gnc::math::Vector3& axis_z) {
    gnc::math::Matrix3 rotation;
    rotation.col(0) = axis_x;
    rotation.col(1) = axis_y;
    rotation.col(2) = axis_z;
    return rotation;
}

inline gnc::math::Matrix3 ecefToInertialRotation(double earth_angle_rad) {
    return rotationZ(earth_angle_rad);
}

inline gnc::math::Matrix3 localGeographicToEarthFixedRotation(double latitude_rad,
                                                              double longitude_rad) {
    const gnc::math::Vector3 north(
        -std::sin(latitude_rad) * std::cos(longitude_rad),
        -std::sin(latitude_rad) * std::sin(longitude_rad),
        std::cos(latitude_rad));
    const gnc::math::Vector3 up(
        std::cos(latitude_rad) * std::cos(longitude_rad),
        std::cos(latitude_rad) * std::sin(longitude_rad),
        std::sin(latitude_rad));
    const gnc::math::Vector3 east(-std::sin(longitude_rad), std::cos(longitude_rad), 0.0);
    return basisToParent(north, up, east);
}

inline gnc::math::Matrix3 launchToLocalGeographicRotation(double azimuth_rad) {
    const gnc::math::Vector3 axis_x(std::cos(azimuth_rad), 0.0, std::sin(azimuth_rad));
    const gnc::math::Vector3 axis_y(0.0, 1.0, 0.0);
    const gnc::math::Vector3 axis_z(-std::sin(azimuth_rad), 0.0, std::cos(azimuth_rad));
    return basisToParent(axis_x, axis_y, axis_z);
}

inline gnc::math::Matrix3 launchInertialToLaunchRotation(double launch_time_s,
                                                         double current_time_s,
                                                         double earth_rotation_rate_radps) {
    return rotationY(earth_rotation_rate_radps * (current_time_s - launch_time_s));
}

inline gnc::math::Matrix3 trackToLaunchRotation(
    const gnc::math::Vector3& velocity_launch_frame) {
    const double speed = velocity_launch_frame.norm();
    if (speed < 1e-9) {
        throw std::runtime_error("Track coordinate system requires a non-zero launch-frame velocity.");
    }

    const gnc::math::Vector3 axis_x = velocity_launch_frame.normalized();
    gnc::math::Vector3 reference_up(0.0, 1.0, 0.0);
    gnc::math::Vector3 axis_z = axis_x.cross(reference_up);
    if (axis_z.norm() < 1e-9) {
        axis_z = gnc::math::Vector3(0.0, 0.0, 1.0);
    }
    axis_z.normalize();
    const gnc::math::Vector3 axis_y = axis_z.cross(axis_x).normalized();
    return basisToParent(axis_x, axis_y, axis_z);
}

inline gnc::math::Matrix3 airflowToBodyRotation(
    const gnc::math::Vector3& airspeed_body_frame) {
    const double speed = airspeed_body_frame.norm();
    if (speed < 1e-9) {
        throw std::runtime_error("Airflow coordinate system requires a non-zero body-frame airspeed.");
    }

    const gnc::math::Vector3 axis_x = airspeed_body_frame.normalized();
    gnc::math::Vector3 reference_up(0.0, 1.0, 0.0);
    gnc::math::Vector3 axis_z = axis_x.cross(reference_up);
    if (axis_z.norm() < 1e-9) {
        axis_z = gnc::math::Vector3(0.0, 0.0, 1.0);
    }
    axis_z.normalize();
    const gnc::math::Vector3 axis_y = axis_z.cross(axis_x).normalized();
    return basisToParent(axis_x, axis_y, axis_z);
}

} // namespace gnc::services::soviet_coord::internal
