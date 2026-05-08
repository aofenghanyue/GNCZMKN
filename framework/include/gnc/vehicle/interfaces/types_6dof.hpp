#pragma once

#include "gnc/common/math/eigen_types.hpp"

namespace gnc::vehicle {

struct ControlSurfaceState6Dof {
    gnc::math::Vector3 fin_deflection_rad = gnc::math::Vector3::Zero();
};

struct AeroQuery6Dof {
    double alpha_rad = 0.0;
    double beta_rad = 0.0;
    double mach_number = 0.0;
    ControlSurfaceState6Dof control_surfaces{};
};

struct AeroCoefficients6Dof {
    double cx = 0.0;
    double cy = 0.0;
    double cz = 0.0;
    double cl = 0.0;
    double cm = 0.0;
    double cn = 0.0;
};

struct ForceMoment6Dof {
    gnc::math::Vector3 force_body_n = gnc::math::Vector3::Zero();
    gnc::math::Vector3 moment_body_nm = gnc::math::Vector3::Zero();
};

struct MassProperties6Dof {
    double mass_kg = 1.0;
    gnc::math::Vector3 center_of_gravity_body_m = gnc::math::Vector3::Zero();
    gnc::math::Matrix3 inertia_body_kgm2 = gnc::math::Matrix3::Identity();
};

} // namespace gnc::vehicle
