#pragma once

namespace gnc::plugins::environment {

struct AtmosphereSample {
    double density_kg_per_m3 = 0.0;
    double pressure_pa = 0.0;
    double temperature_k = 0.0;
    double speed_of_sound_mps = 0.0;
};

class IAtmosphere {
public:
    virtual ~IAtmosphere() = default;

    virtual AtmosphereSample sample(double altitude_m) const = 0;
    virtual double getSeaLevelDensity() const = 0;
    virtual double getSeaLevelPressure() const = 0;
    virtual double getSeaLevelTemperature() const = 0;
    virtual double getSeaLevelSpeedOfSound() const = 0;

    virtual double getDensity(double altitude_m) const {
        return sample(altitude_m).density_kg_per_m3;
    }

    virtual double getPressure(double altitude_m) const {
        return sample(altitude_m).pressure_pa;
    }

    virtual double getTemperature(double altitude_m) const {
        return sample(altitude_m).temperature_k;
    }

    virtual double getSpeedOfSound(double altitude_m) const {
        return sample(altitude_m).speed_of_sound_mps;
    }
};

} // namespace gnc::plugins::environment
