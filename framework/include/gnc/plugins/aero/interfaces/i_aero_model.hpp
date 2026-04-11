#pragma once

#include "gnc/plugins/aero/interfaces/aero_coefficients.hpp"

namespace gnc::plugins::aero {

class IAeroModel {
public:
    virtual ~IAeroModel() = default;

    virtual AeroCoefficients computeCoefficients(double angle_of_attack_rad,
                                                 double sideslip_angle_rad,
                                                 double mach_number) const = 0;
    virtual double getReferenceArea() const = 0;
    virtual double getReferenceLength() const = 0;
};

} // namespace gnc::plugins::aero
