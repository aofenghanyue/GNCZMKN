#pragma once

#include "gnc/vehicle/output/interfaces/aero_coefficients.hpp"

namespace gnc::vehicle::common {

struct AeroQuery3Dof {
    double angle_of_attack_rad = 0.0;
    double sideslip_angle_rad = 0.0;
    double mach_number = 0.0;
};

class IAerodynamicAssets3Dof {
public:
    virtual ~IAerodynamicAssets3Dof() = default;

    virtual gnc::vehicle::output::AeroCoefficients sampleAeroCoefficients3Dof(
        const AeroQuery3Dof& query) const = 0;
};

} // namespace gnc::vehicle::common
