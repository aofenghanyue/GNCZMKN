#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/plugins/environment/interfaces/i_atmosphere.hpp"

#include <algorithm>
#include <cmath>

namespace gnc::plugins::environment {

class StandardAtmosphere final : public gnc::core::ComponentBase,
                                 public IAtmosphere {
public:
    StandardAtmosphere() : ComponentBase("StandardAtmosphere") {
        initializeLayers();
    }

    void update(double) override {}

    double getDensity(double altitude_m) const override {
        return getPressure(altitude_m) /
               (kSpecificGasConstant * getTemperature(altitude_m));
    }

    double getPressure(double altitude_m) const override {
        const double altitude = std::clamp(altitude_m, 0.0, 86000.0);
        const Layer& layer = layers_[findLayer(altitude)];
        const double delta_h = altitude - layer.base_altitude_m;

        if (std::abs(layer.lapse_rate_k_per_m) < 1e-10) {
            return layer.base_pressure_pa *
                   std::exp(-kSeaLevelGravity * delta_h /
                            (kSpecificGasConstant * layer.base_temperature_k));
        }

        const double temperature =
            layer.base_temperature_k + layer.lapse_rate_k_per_m * delta_h;
        const double exponent =
            -kSeaLevelGravity / (layer.lapse_rate_k_per_m * kSpecificGasConstant);
        return layer.base_pressure_pa *
               std::pow(temperature / layer.base_temperature_k, exponent);
    }

    double getTemperature(double altitude_m) const override {
        const double altitude = std::clamp(altitude_m, 0.0, 86000.0);
        const Layer& layer = layers_[findLayer(altitude)];
        return layer.base_temperature_k +
               layer.lapse_rate_k_per_m * (altitude - layer.base_altitude_m);
    }

    double getSpeedOfSound(double altitude_m) const override {
        return std::sqrt(kHeatCapacityRatio * kSpecificGasConstant *
                         getTemperature(altitude_m));
    }

private:
    struct Layer {
        double base_altitude_m = 0.0;
        double base_temperature_k = 0.0;
        double lapse_rate_k_per_m = 0.0;
        double base_pressure_pa = 0.0;
    };

    static constexpr double kSeaLevelGravity = 9.80665;
    static constexpr double kSpecificGasConstant = 287.05287;
    static constexpr double kHeatCapacityRatio = 1.4;
    static constexpr int kLayerCount = 7;

    void initializeLayers() {
        constexpr double base_altitudes[kLayerCount] = {
            0.0, 11000.0, 20000.0, 32000.0, 47000.0, 51000.0, 71000.0};
        constexpr double base_temperatures[kLayerCount] = {
            288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65};
        constexpr double lapse_rates[kLayerCount] = {
            -0.0065, 0.0, 0.0010, 0.0028, 0.0, -0.0028, -0.0020};

        layers_[0] = {base_altitudes[0], base_temperatures[0], lapse_rates[0], 101325.0};
        for (int i = 1; i < kLayerCount; ++i) {
            const Layer& previous = layers_[i - 1];
            const double delta_h = base_altitudes[i] - previous.base_altitude_m;
            double pressure = 0.0;

            if (std::abs(previous.lapse_rate_k_per_m) < 1e-10) {
                pressure = previous.base_pressure_pa *
                           std::exp(-kSeaLevelGravity * delta_h /
                                    (kSpecificGasConstant * previous.base_temperature_k));
            } else {
                const double temperature =
                    previous.base_temperature_k + previous.lapse_rate_k_per_m * delta_h;
                const double exponent =
                    -kSeaLevelGravity /
                    (previous.lapse_rate_k_per_m * kSpecificGasConstant);
                pressure = previous.base_pressure_pa *
                           std::pow(temperature / previous.base_temperature_k, exponent);
            }

            layers_[i] = {base_altitudes[i], base_temperatures[i], lapse_rates[i], pressure};
        }
    }

    int findLayer(double altitude_m) const {
        for (int i = kLayerCount - 1; i > 0; --i) {
            if (altitude_m >= layers_[i].base_altitude_m) {
                return i;
            }
        }
        return 0;
    }

    Layer layers_[kLayerCount]{};
};

} // namespace gnc::plugins::environment
