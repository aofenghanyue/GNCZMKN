#pragma once

namespace gnc::vehicle::common {

class IContinuousMass {
public:
    virtual ~IContinuousMass() = default;

    virtual double getMassKg() const = 0;
    virtual double getMassRateKgPerSec() const = 0;
};

} // namespace gnc::vehicle::common
