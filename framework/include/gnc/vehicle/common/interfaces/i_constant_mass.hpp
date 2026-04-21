#pragma once

namespace gnc::vehicle::common {

class IConstantMass {
public:
    virtual ~IConstantMass() = default;

    virtual double getMassKg() const = 0;
};

} // namespace gnc::vehicle::common
