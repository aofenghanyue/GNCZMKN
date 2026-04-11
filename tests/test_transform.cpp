/**
 * @file test_transform.cpp
 * @brief Numerical checks for the built-in Soviet coordinate system entry point.
 */

#include "gnc/common/math/math.hpp"
#include "gnc/services/coordinate/soviet_coordinate_system.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

using gnc::math::Matrix3;
using gnc::math::Quaternion;
using gnc::math::Vector3;
namespace soviet = gnc::services::coordinate::soviet;

namespace {

constexpr double kTol = 1e-9;

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool nearlyEqual(double lhs, double rhs, double tol = kTol) {
    return std::abs(lhs - rhs) <= tol;
}

bool matrixClose(const Matrix3& lhs, const Matrix3& rhs, double tol = kTol) {
    return (lhs - rhs).cwiseAbs().maxCoeff() <= tol;
}

bool vectorClose(const Vector3& lhs, const Vector3& rhs, double tol = kTol) {
    return (lhs - rhs).cwiseAbs().maxCoeff() <= tol;
}

void testEarthAndLaunchFrames() {
    const double latitude = gnc::math::deg2rad(40.0);
    const double longitude = gnc::math::deg2rad(100.0);
    const double azimuth = gnc::math::deg2rad(30.0);

    const Matrix3 ecef_to_nue = soviet::earthFixedToLocalNueRotation(latitude, longitude);
    const Matrix3 nue_to_ecef = soviet::localNueToEarthFixedRotation(latitude, longitude);
    require(matrixClose(ecef_to_nue * nue_to_ecef, Matrix3::Identity()),
            "ECEF<->NUE rotations should be inverses.");

    const Matrix3 nue_to_launch = soviet::localNueToLaunchRotation(azimuth);
    const Matrix3 launch_to_nue = soviet::launchToLocalNueRotation(azimuth);
    require(matrixClose(nue_to_launch * launch_to_nue, Matrix3::Identity()),
            "NUE<->LAUNCH rotations should be inverses.");

    const Matrix3 ecef_to_launch = soviet::earthFixedToLaunchRotation(latitude, longitude, azimuth);
    require(matrixClose(ecef_to_launch, nue_to_launch * ecef_to_nue),
            "ECEF->LAUNCH should equal NUE->LAUNCH composed with ECEF->NUE.");
    require(matrixClose(soviet::launchToEarthFixedRotation(latitude, longitude, azimuth),
                        ecef_to_launch.transpose()),
            "LAUNCH->ECEF should be the transpose of ECEF->LAUNCH.");

    const Matrix3 li_to_l_t0 = soviet::launchInertialToLaunchRotation(0.0, ecef_to_launch, 0.01);
    require(matrixClose(li_to_l_t0, Matrix3::Identity()),
            "LI and L should coincide at t0.");
}

void testBodyTrackAndWindFrames() {
    const Quaternion q_l_b = soviet::eulerToAttitude(
        gnc::math::deg2rad(10.0),
        gnc::math::deg2rad(-5.0),
        gnc::math::deg2rad(3.0));
    const Matrix3 l_to_b = soviet::referenceToBodyRotation(q_l_b);
    const Matrix3 b_to_l = soviet::bodyToReferenceRotation(q_l_b);
    require(matrixClose(l_to_b * b_to_l, Matrix3::Identity()),
            "BODY and LAUNCH rotations should be inverses.");

    const Vector3 recovered = soviet::attitudeToEuler(q_l_b);
    require(nearlyEqual(recovered.x(), gnc::math::deg2rad(10.0)),
            "Attitude->Euler yaw mismatch.");
    require(nearlyEqual(recovered.y(), gnc::math::deg2rad(-5.0)),
            "Attitude->Euler pitch mismatch.");
    require(nearlyEqual(recovered.z(), gnc::math::deg2rad(3.0)),
            "Attitude->Euler roll mismatch.");

    const Vector3 ground_velocity_l(120.0, 15.0, -30.0);
    double chi = 0.0;
    double theta = 0.0;
    soviet::computeTrackAngles(ground_velocity_l, chi, theta);
    const Matrix3 k_to_l = soviet::trackToReferenceRotation(chi, theta);
    const Matrix3 l_to_k = soviet::referenceToTrackRotation(chi, theta);
    require(matrixClose(k_to_l * l_to_k, Matrix3::Identity()),
            "TRACK and LAUNCH rotations should be inverses.");

    const Vector3 launch_x_in_track = l_to_k * Vector3(1.0, 0.0, 0.0);
    require(std::abs(launch_x_in_track.norm() - 1.0) < kTol,
            "TRACK rotation should preserve vector norms.");

    const Vector3 air_velocity_b(200.0, -20.0, 30.0);
    double alpha = 0.0;
    double beta = 0.0;
    soviet::computeWindAngles(air_velocity_b, alpha, beta);
    const Matrix3 v_to_b = soviet::windToBodyRotation(alpha, beta);
    const Matrix3 b_to_v = soviet::bodyToWindRotation(alpha, beta);
    require(matrixClose(v_to_b * b_to_v, Matrix3::Identity()),
            "WIND and BODY rotations should be inverses.");

    const Vector3 round_trip = v_to_b * (b_to_v * Vector3(1.0, 2.0, 3.0));
    require(vectorClose(round_trip, Vector3(1.0, 2.0, 3.0)),
            "WIND/BODY round trip should preserve vectors.");
}

void testInertialEarthPair() {
    const double angle = 0.3;
    const Matrix3 i_to_e = soviet::inertialToEarthFixedRotation(angle);
    const Matrix3 e_to_i = soviet::earthFixedToInertialRotation(angle);
    require(matrixClose(i_to_e * e_to_i, Matrix3::Identity()),
            "ECI/ECEF rotations should be inverses.");
}

} // namespace

int main() {
    try {
        std::cout << std::scientific << std::setprecision(8);
        testInertialEarthPair();
        testEarthAndLaunchFrames();
        testBodyTrackAndWindFrames();
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }

    std::cout << "Soviet transform tests passed\n";
    return 0;
}
