#pragma once

namespace gnc::vehicle::output {

class IConstantMass {
public:
    virtual ~IConstantMass() = default;
    virtual double getMassKg() const = 0;
};

} // namespace gnc::vehicle::output
