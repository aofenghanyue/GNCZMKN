#pragma once

namespace gnc::vehicle::input {

struct AirDataMeasurement6Dof {
    double altitude_m = 0.0;
    double speed_mps = 0.0;
    double density_kg_per_m3 = 0.0;
    double pressure_pa = 0.0;
    double temperature_k = 0.0;
    double speed_of_sound_mps = 0.0;
    double mach_number = 0.0;
};

class IAirData6Dof {
public:
    virtual ~IAirData6Dof() = default;

    virtual const AirDataMeasurement6Dof& airDataMeasurement6Dof() const = 0;
};

} // namespace gnc::vehicle::input
