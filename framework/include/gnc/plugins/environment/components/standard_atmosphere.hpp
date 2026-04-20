#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/plugins/environment/interfaces/i_atmosphere.hpp"
#include "gnc_generated_ussa1976_atmosphere.hpp"

#include <algorithm>
#include <cstddef>

namespace gnc::plugins::environment {

class StandardAtmosphere final : public gnc::core::ComponentBase,
                                 public IAtmosphere {
public:
    StandardAtmosphere() : ComponentBase("StandardAtmosphere") {}

    void update(double) override {}

    AtmosphereSample sample(double altitude_m) const override {
        const double altitude_km =
            std::clamp(altitude_m * 1.0e-3,
                       gnc::generated::kUssa1976AltitudeKm.front(),
                       gnc::generated::kUssa1976AltitudeKm.back());

        const auto lower =
            std::lower_bound(gnc::generated::kUssa1976AltitudeKm.begin(),
                             gnc::generated::kUssa1976AltitudeKm.end(),
                             altitude_km);
        if (lower == gnc::generated::kUssa1976AltitudeKm.begin()) {
            return sampleAtIndex(0);
        }
        if (lower == gnc::generated::kUssa1976AltitudeKm.end()) {
            return sampleAtIndex(gnc::generated::kUssa1976TableSize - 1);
        }

        const std::size_t upper_index =
            static_cast<std::size_t>(lower - gnc::generated::kUssa1976AltitudeKm.begin());
        const std::size_t lower_index = upper_index - 1;
        const double lower_altitude = gnc::generated::kUssa1976AltitudeKm[lower_index];
        const double upper_altitude = gnc::generated::kUssa1976AltitudeKm[upper_index];
        const double blend =
            (altitude_km - lower_altitude) / (upper_altitude - lower_altitude);

        AtmosphereSample interpolated;
        interpolated.temperature_k =
            lerp(gnc::generated::kUssa1976TemperatureK[lower_index],
                 gnc::generated::kUssa1976TemperatureK[upper_index],
                 blend);
        interpolated.pressure_pa =
            lerp(gnc::generated::kUssa1976PressurePa[lower_index],
                 gnc::generated::kUssa1976PressurePa[upper_index],
                 blend);
        interpolated.density_kg_per_m3 =
            lerp(gnc::generated::kUssa1976DensityKgPerM3[lower_index],
                 gnc::generated::kUssa1976DensityKgPerM3[upper_index],
                 blend);
        interpolated.speed_of_sound_mps =
            lerp(gnc::generated::kUssa1976SpeedOfSoundMps[lower_index],
                 gnc::generated::kUssa1976SpeedOfSoundMps[upper_index],
                 blend);
        return interpolated;
    }

    double getSeaLevelDensity() const override {
        return gnc::generated::kUssa1976DensityKgPerM3.front();
    }

    double getSeaLevelPressure() const override {
        return gnc::generated::kUssa1976PressurePa.front();
    }

    double getSeaLevelTemperature() const override {
        return gnc::generated::kUssa1976TemperatureK.front();
    }

    double getSeaLevelSpeedOfSound() const override {
        return gnc::generated::kUssa1976SpeedOfSoundMps.front();
    }

private:
    static double lerp(double x0, double x1, double t) {
        return x0 + (x1 - x0) * t;
    }

    static AtmosphereSample sampleAtIndex(std::size_t index) {
        AtmosphereSample sample;
        sample.temperature_k = gnc::generated::kUssa1976TemperatureK[index];
        sample.pressure_pa = gnc::generated::kUssa1976PressurePa[index];
        sample.density_kg_per_m3 = gnc::generated::kUssa1976DensityKgPerM3[index];
        sample.speed_of_sound_mps = gnc::generated::kUssa1976SpeedOfSoundMps[index];
        return sample;
    }
};

} // namespace gnc::plugins::environment
