#pragma once

namespace gnc::plugins::environment {

class IAtmosphere {
public:
    virtual ~IAtmosphere() = default;

    virtual double getDensity(double altitude_m) const = 0;
    virtual double getPressure(double altitude_m) const = 0;
    virtual double getTemperature(double altitude_m) const = 0;
    virtual double getSpeedOfSound(double altitude_m) const = 0;
};

} // namespace gnc::plugins::environment
