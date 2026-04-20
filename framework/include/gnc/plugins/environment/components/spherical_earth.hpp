#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/plugins/environment/interfaces/i_earth.hpp"

#include <cmath>

namespace gnc::plugins::environment {

class SphericalEarth final : public gnc::core::ComponentBase,
                             public IEarth {
public:
    SphericalEarth() : ComponentBase("SphericalEarth") {}

    void configure(const gnc::core::ConfigNode& config) override {
        if (config.isNull()) {
            return;
        }
        equatorial_radius_m_ =
            config["equatorial_radius_m"].asDouble(equatorial_radius_m_);
        rotation_rate_rad_per_s_ =
            config["rotation_rate_rad_per_s"].asDouble(rotation_rate_rad_per_s_);
    }

    void update(double) override {}

    double getEquatorialRadius() const override { return equatorial_radius_m_; }
    double getFlattening() const override { return 0.0; }
    double getRotationRate() const override { return rotation_rate_rad_per_s_; }

    gnc::math::Vector3 geodeticToEcef(double latitude_rad,
                                      double longitude_rad,
                                      double altitude_m) const override {
        const double radius = equatorial_radius_m_ + altitude_m;
        const double sin_lat = std::sin(latitude_rad);
        const double cos_lat = std::cos(latitude_rad);
        const double sin_lon = std::sin(longitude_rad);
        const double cos_lon = std::cos(longitude_rad);

        return gnc::math::Vector3(radius * cos_lat * cos_lon,
                                  radius * cos_lat * sin_lon,
                                  radius * sin_lat);
    }

private:
    double equatorial_radius_m_ = 6371000.0;
    double rotation_rate_rad_per_s_ = 7.292115e-5;
};

} // namespace gnc::plugins::environment
