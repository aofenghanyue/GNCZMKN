#pragma once

#include "gnc/interfaces/vehicle/aero_types.hpp"

namespace gnc::interfaces {

class IAeroCoefficients {
public:
    virtual ~IAeroCoefficients() = default;
    virtual AeroCoefficients computeCoefficients(double alpha,
                                                 double beta,
                                                 double mach) const = 0;
    virtual double getReferenceArea() const = 0;
    virtual double getReferenceLength() const = 0;
};

} // namespace gnc::interfaces
