#pragma once

#include "gnc/forms/local_spherical_3dof/internal/math.hpp"
#include "gnc/plugins/state_3dof/interfaces/soviet_spherical_state_3dof.hpp"

namespace gnc::plugins::state_3dof::internal {

using SovietSphericalStateDerivative3DOF =
    ::gnc::forms::local_spherical_3dof::StateDerivative;

inline ::gnc::forms::local_spherical_3dof::State toFormState(
    const SovietSphericalState3DOF& state) {
    ::gnc::forms::local_spherical_3dof::State converted;
    converted.longitude_rad = state.longitude_rad;
    converted.latitude_rad = state.latitude_rad;
    converted.altitude_m = state.altitude_m;
    converted.speed_mps = state.speed_mps;
    converted.flight_path_angle_rad = state.flight_path_angle_rad;
    converted.heading_angle_rad = state.heading_angle_rad;
    return converted;
}

inline double clampSigned(double value, double min_abs = 1.0e-6) {
    return ::gnc::forms::local_spherical_3dof::internal::clampSigned(value, min_abs);
}

inline double sphericalRadius(double earth_radius_m, double altitude_m) {
    return ::gnc::forms::local_spherical_3dof::internal::sphericalRadius(
        earth_radius_m,
        altitude_m);
}

inline gnc::math::Vector3 localVelocityNue(const SovietSphericalState3DOF& state) {
    return ::gnc::forms::local_spherical_3dof::internal::localVelocityNue(
        toFormState(state));
}

inline gnc::math::Vector3 velocityBasisK1(const SovietSphericalState3DOF& state) {
    return ::gnc::forms::local_spherical_3dof::internal::velocityBasisK1(
        toFormState(state));
}

inline gnc::math::Vector3 velocityBasisK2(const SovietSphericalState3DOF& state) {
    return ::gnc::forms::local_spherical_3dof::internal::velocityBasisK2(
        toFormState(state));
}

inline gnc::math::Vector3 velocityBasisK3(const SovietSphericalState3DOF& state) {
    return ::gnc::forms::local_spherical_3dof::internal::velocityBasisK3(
        toFormState(state));
}

inline gnc::math::Vector3 northAxisEcef(double latitude_rad, double longitude_rad) {
    return ::gnc::forms::local_spherical_3dof::internal::northAxisEcef(
        latitude_rad,
        longitude_rad);
}

inline gnc::math::Vector3 upAxisEcef(double latitude_rad, double longitude_rad) {
    return ::gnc::forms::local_spherical_3dof::internal::upAxisEcef(
        latitude_rad,
        longitude_rad);
}

inline gnc::math::Vector3 eastAxisEcef(double latitude_rad, double longitude_rad) {
    return ::gnc::forms::local_spherical_3dof::internal::eastAxisEcef(
        latitude_rad,
        longitude_rad);
}

inline gnc::math::Matrix3 localNueToEcefRotation(double latitude_rad,
                                                  double longitude_rad) {
    return ::gnc::forms::local_spherical_3dof::internal::localNueToEcefRotation(
        latitude_rad,
        longitude_rad);
}

inline SovietSphericalStateDerivative3DOF computePositionRates(
    const SovietSphericalState3DOF& state,
    double earth_radius_m) {
    return ::gnc::forms::local_spherical_3dof::internal::computePositionRates(
        toFormState(state),
        earth_radius_m);
}

inline gnc::math::Vector3 quadraticRotationTermNue(
    const SovietSphericalState3DOF& state,
    double earth_radius_m,
    double earth_rotation_rate_rad_per_s) {
    return ::gnc::forms::local_spherical_3dof::internal::quadraticRotationTermNue(
        toFormState(state),
        earth_radius_m,
        earth_rotation_rate_rad_per_s);
}

inline SovietSphericalStateDerivative3DOF computeStateDerivatives(
    const SovietSphericalState3DOF& state,
    const gnc::math::Vector3& local_acceleration_nue_m_per_s2,
    double earth_radius_m,
    double earth_rotation_rate_rad_per_s) {
    return ::gnc::forms::local_spherical_3dof::internal::computeStateDerivatives(
        toFormState(state),
        local_acceleration_nue_m_per_s2,
        earth_radius_m,
        earth_rotation_rate_rad_per_s);
}

inline gnc::math::Vector3 earthRelativeVelocityDerivativeNue(
    const SovietSphericalState3DOF& state,
    const SovietSphericalStateDerivative3DOF& derivative) {
    return ::gnc::forms::local_spherical_3dof::internal::earthRelativeVelocityDerivativeNue(
        toFormState(state),
        derivative);
}

inline gnc::math::Vector3 reconstructLocalAccelerationNue(
    const SovietSphericalState3DOF& state,
    const SovietSphericalStateDerivative3DOF& derivative,
    double earth_radius_m,
    double earth_rotation_rate_rad_per_s) {
    return ::gnc::forms::local_spherical_3dof::internal::reconstructLocalAccelerationNue(
        toFormState(state),
        derivative,
        earth_radius_m,
        earth_rotation_rate_rad_per_s);
}

} // namespace gnc::plugins::state_3dof::internal
