#pragma once

namespace gnc::plugins::mass {

class IContinuousMass {
public:
    virtual ~IContinuousMass() = default;

    virtual double getMassKg() const = 0;
    virtual double getMassRateKgPerSec() const = 0;
};

} // namespace gnc::plugins::mass
