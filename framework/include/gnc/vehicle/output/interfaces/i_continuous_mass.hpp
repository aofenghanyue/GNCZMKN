#pragma once

namespace gnc::vehicle::output {

class IContinuousMass {
public:
    virtual ~IContinuousMass() = default;
    virtual double getMassKg() const = 0;
    virtual double getMassRateKgPerSec() const = 0;
};

} // namespace gnc::vehicle::output
