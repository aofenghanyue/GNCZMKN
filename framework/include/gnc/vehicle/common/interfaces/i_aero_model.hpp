#pragma once

#include "gnc/vehicle/common/interfaces/aero_coefficients.hpp"

namespace gnc::vehicle::common {

class IAeroModel {
public:
    virtual ~IAeroModel() = default;

    virtual AeroCoefficients computeCoefficients(double angle_of_attack_rad,
                                                 double sideslip_angle_rad,
                                                 double mach_number) const = 0;
    virtual double getReferenceArea() const = 0;
    virtual double getReferenceLength() const = 0;
};

} // namespace gnc::vehicle::common
