#pragma once

namespace gnc::vehicle::input {

struct AirDataMeasurement3Dof {
    double altitude_m = 0.0;
    double speed_mps = 0.0;
    double density_kg_per_m3 = 0.0;
    double pressure_pa = 0.0;
    double temperature_k = 0.0;
    double speed_of_sound_mps = 0.0;
    double mach_number = 0.0;
    double dynamic_pressure_pa = 0.0;
};

class IAirData3Dof {
public:
    virtual ~IAirData3Dof() = default;
    virtual const AirDataMeasurement3Dof& airDataMeasurement3Dof() const = 0;
};

} // namespace gnc::vehicle::input
