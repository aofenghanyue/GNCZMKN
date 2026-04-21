#pragma once

#include "gnc/common/math/eigen_types.hpp"
#include "gnc/forms/local_spherical_3dof/types.hpp"

#include <algorithm>
#include <cmath>

namespace gnc::forms::local_spherical_3dof::internal {

inline double clampSigned(double value, double min_abs = 1.0e-6) {
    if (std::abs(value) >= min_abs) {
        return value;
    }
    return value >= 0.0 ? min_abs : -min_abs;
}

inline double sphericalRadius(double earth_radius_m, double altitude_m) {
    return earth_radius_m + altitude_m;
}

inline gnc::math::Vector3 localVelocityNue(const State& state) {
    const double cos_gamma = std::cos(state.flight_path_angle_rad);
    const double sin_gamma = std::sin(state.flight_path_angle_rad);
    const double cos_heading = std::cos(state.heading_angle_rad);
    const double sin_heading = std::sin(state.heading_angle_rad);

    return gnc::math::Vector3(state.speed_mps * cos_gamma * cos_heading,
                              state.speed_mps * sin_gamma,
                              -state.speed_mps * cos_gamma * sin_heading);
}

inline gnc::math::Vector3 velocityBasisK1(const State& state) {
    const double cos_gamma = std::cos(state.flight_path_angle_rad);
    const double sin_gamma = std::sin(state.flight_path_angle_rad);
    const double cos_heading = std::cos(state.heading_angle_rad);
    const double sin_heading = std::sin(state.heading_angle_rad);

    return gnc::math::Vector3(cos_gamma * cos_heading,
                              sin_gamma,
                              -cos_gamma * sin_heading);
}

inline gnc::math::Vector3 velocityBasisK2(const State& state) {
    const double cos_gamma = std::cos(state.flight_path_angle_rad);
    const double sin_gamma = std::sin(state.flight_path_angle_rad);
    const double cos_heading = std::cos(state.heading_angle_rad);
    const double sin_heading = std::sin(state.heading_angle_rad);

    return gnc::math::Vector3(-sin_gamma * cos_heading,
                              cos_gamma,
                              sin_gamma * sin_heading);
}

inline gnc::math::Vector3 velocityBasisK3(const State& state) {
    return gnc::math::Vector3(std::sin(state.heading_angle_rad),
                              0.0,
                              std::cos(state.heading_angle_rad));
}

inline gnc::math::Vector3 northAxisEcef(double latitude_rad, double longitude_rad) {
    return gnc::math::Vector3(-std::sin(latitude_rad) * std::cos(longitude_rad),
                              -std::sin(latitude_rad) * std::sin(longitude_rad),
                              std::cos(latitude_rad));
}

inline gnc::math::Vector3 upAxisEcef(double latitude_rad, double longitude_rad) {
    return gnc::math::Vector3(std::cos(latitude_rad) * std::cos(longitude_rad),
                              std::cos(latitude_rad) * std::sin(longitude_rad),
                              std::sin(latitude_rad));
}

inline gnc::math::Vector3 eastAxisEcef(double, double longitude_rad) {
    return gnc::math::Vector3(-std::sin(longitude_rad),
                              std::cos(longitude_rad),
                              0.0);
}

inline gnc::math::Matrix3 localNueToEcefRotation(double latitude_rad,
                                                  double longitude_rad) {
    gnc::math::Matrix3 rotation;
    rotation.col(0) = northAxisEcef(latitude_rad, longitude_rad);
    rotation.col(1) = upAxisEcef(latitude_rad, longitude_rad);
    rotation.col(2) = eastAxisEcef(latitude_rad, longitude_rad);
    return rotation;
}

inline StateDerivative computePositionRates(const State& state, double earth_radius_m) {
    StateDerivative derivative;

    const double radius = sphericalRadius(earth_radius_m, state.altitude_m);
    const double cos_gamma = std::cos(state.flight_path_angle_rad);
    const double cos_heading = std::cos(state.heading_angle_rad);
    const double sin_heading = std::sin(state.heading_angle_rad);
    const double cos_latitude = clampSigned(std::cos(state.latitude_rad));

    derivative.latitude_rate_rad_per_s =
        state.speed_mps * cos_gamma * cos_heading / radius;
    derivative.longitude_rate_rad_per_s =
        -state.speed_mps * cos_gamma * sin_heading / (radius * cos_latitude);
    derivative.altitude_rate_m_per_s =
        state.speed_mps * std::sin(state.flight_path_angle_rad);
    return derivative;
}

inline gnc::math::Vector3 quadraticRotationTermNue(
    const State& state,
    double earth_radius_m,
    double earth_rotation_rate_rad_per_s) {
    const double radius = sphericalRadius(earth_radius_m, state.altitude_m);
    const double sin_latitude = std::sin(state.latitude_rad);
    const double cos_latitude = std::cos(state.latitude_rad);
    const double omega2 = earth_rotation_rate_rad_per_s * earth_rotation_rate_rad_per_s;

    return gnc::math::Vector3(omega2 * radius * sin_latitude * cos_latitude,
                              omega2 * radius * cos_latitude * cos_latitude,
                              0.0);
}

inline StateDerivative computeStateDerivatives(
    const State& state,
    const gnc::math::Vector3& local_acceleration_nue_m_per_s2,
    double earth_radius_m,
    double earth_rotation_rate_rad_per_s) {
    StateDerivative derivative = computePositionRates(state, earth_radius_m);

    const double speed = std::max(1.0, state.speed_mps);
    const double sin_gamma = std::sin(state.flight_path_angle_rad);
    const double cos_gamma = std::cos(state.flight_path_angle_rad);
    const double tan_gamma = std::tan(state.flight_path_angle_rad);
    const double cos_gamma_safe = clampSigned(cos_gamma);
    const double sin_heading = std::sin(state.heading_angle_rad);
    const double cos_heading = std::cos(state.heading_angle_rad);
    const double sin_latitude = std::sin(state.latitude_rad);
    const double cos_latitude = std::cos(state.latitude_rad);
    const gnc::math::Vector3 q_omega =
        quadraticRotationTermNue(state, earth_radius_m, earth_rotation_rate_rad_per_s);

    const double coupled_longitude_rate =
        derivative.longitude_rate_rad_per_s + 2.0 * earth_rotation_rate_rad_per_s;

    derivative.speed_rate_m_per_s2 =
        local_acceleration_nue_m_per_s2.x() * cos_gamma * cos_heading +
        local_acceleration_nue_m_per_s2.y() * sin_gamma -
        local_acceleration_nue_m_per_s2.z() * cos_gamma * sin_heading -
        q_omega.x() * cos_gamma * cos_heading - q_omega.y() * sin_gamma +
        q_omega.z() * cos_gamma * sin_heading;

    derivative.flight_path_angle_rate_rad_per_s =
        (-local_acceleration_nue_m_per_s2.x() * sin_gamma * cos_heading +
         local_acceleration_nue_m_per_s2.y() * cos_gamma +
         local_acceleration_nue_m_per_s2.z() * sin_gamma * sin_heading) /
            speed +
        derivative.latitude_rate_rad_per_s * cos_heading -
        coupled_longitude_rate * sin_heading * cos_latitude +
        (q_omega.x() * sin_gamma * cos_heading -
         q_omega.y() * cos_gamma -
         q_omega.z() * sin_gamma * sin_heading) /
            speed;

    derivative.heading_angle_rate_rad_per_s =
        -(local_acceleration_nue_m_per_s2.x() * sin_heading +
          local_acceleration_nue_m_per_s2.z() * cos_heading) /
            (speed * cos_gamma_safe) +
        derivative.latitude_rate_rad_per_s * tan_gamma * sin_heading +
        coupled_longitude_rate *
            (tan_gamma * cos_heading * cos_latitude - sin_latitude) +
        (q_omega.x() * sin_heading + q_omega.z() * cos_heading) /
            (speed * cos_gamma_safe);

    return derivative;
}

inline gnc::math::Vector3 earthRelativeVelocityDerivativeNue(
    const State& state,
    const StateDerivative& derivative) {
    return derivative.speed_rate_m_per_s2 * velocityBasisK1(state) +
           state.speed_mps * derivative.flight_path_angle_rate_rad_per_s *
               velocityBasisK2(state) -
           state.speed_mps * std::cos(state.flight_path_angle_rad) *
               derivative.heading_angle_rate_rad_per_s * velocityBasisK3(state);
}

inline gnc::math::Vector3 reconstructLocalAccelerationNue(
    const State& state,
    const StateDerivative& derivative,
    double earth_radius_m,
    double earth_rotation_rate_rad_per_s) {
    const gnc::math::Vector3 w_nue =
        earthRelativeVelocityDerivativeNue(state, derivative);
    const gnc::math::Vector3 q_omega =
        quadraticRotationTermNue(state, earth_radius_m, earth_rotation_rate_rad_per_s);

    const double coupled_longitude_rate =
        derivative.longitude_rate_rad_per_s + 2.0 * earth_rotation_rate_rad_per_s;
    const double sin_gamma = std::sin(state.flight_path_angle_rad);
    const double cos_gamma = std::cos(state.flight_path_angle_rad);
    const double sin_heading = std::sin(state.heading_angle_rad);
    const double cos_heading = std::cos(state.heading_angle_rad);
    const double sin_latitude = std::sin(state.latitude_rad);
    const double cos_latitude = std::cos(state.latitude_rad);

    gnc::math::Vector3 acceleration_nue;
    acceleration_nue.x() =
        w_nue.x() + state.speed_mps * derivative.latitude_rate_rad_per_s * sin_gamma -
        state.speed_mps * cos_gamma * sin_heading * sin_latitude *
            coupled_longitude_rate +
        q_omega.x();
    acceleration_nue.y() =
        w_nue.y() - state.speed_mps * cos_gamma *
                         derivative.latitude_rate_rad_per_s * cos_heading +
        state.speed_mps * cos_gamma * sin_heading * cos_latitude *
            coupled_longitude_rate +
        q_omega.y();
    acceleration_nue.z() =
        w_nue.z() +
        state.speed_mps * coupled_longitude_rate *
            (sin_gamma * cos_latitude -
             sin_latitude * cos_heading * cos_gamma) +
        q_omega.z();
    return acceleration_nue;
}

inline gnc::math::Vector3 velocityLaunchFrame(const Truth& truth) {
    const double cos_azimuth = std::cos(truth.launch_azimuth_rad);
    const double sin_azimuth = std::sin(truth.launch_azimuth_rad);

    return gnc::math::Vector3(
        truth.local_velocity_nue_mps.x() * cos_azimuth +
            truth.local_velocity_nue_mps.z() * sin_azimuth,
        truth.local_velocity_nue_mps.y(),
        -truth.local_velocity_nue_mps.x() * sin_azimuth +
            truth.local_velocity_nue_mps.z() * cos_azimuth);
}

} // namespace gnc::forms::local_spherical_3dof::internal
