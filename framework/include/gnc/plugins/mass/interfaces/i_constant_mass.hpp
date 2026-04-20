#pragma once

namespace gnc::plugins::mass {

class IConstantMass {
public:
    virtual ~IConstantMass() = default;

    virtual double getMassKg() const = 0;
};

} // namespace gnc::plugins::mass
