#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/plugins/environment/interfaces/i_earth.hpp"

#include <cmath>

namespace gnc::plugins::environment {

class Wgs84Earth final : public gnc::core::ComponentBase,
                         public IEarth {
public:
    Wgs84Earth() : ComponentBase("Wgs84Earth") {}

    void update(double) override {}

    double getEquatorialRadius() const override { return 6378137.0; }
    double getFlattening() const override { return 1.0 / 298.257223563; }
    double getRotationRate() const override { return 7.292115e-5; }

    gnc::math::Vector3 geodeticToEcef(double latitude_rad,
                                      double longitude_rad,
                                      double altitude_m) const override {
        const double a = getEquatorialRadius();
        const double f = getFlattening();
        const double e2 = f * (2.0 - f);

        const double sin_lat = std::sin(latitude_rad);
        const double cos_lat = std::cos(latitude_rad);
        const double sin_lon = std::sin(longitude_rad);
        const double cos_lon = std::cos(longitude_rad);

        const double prime_vertical =
            a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
        return gnc::math::Vector3(
            (prime_vertical + altitude_m) * cos_lat * cos_lon,
            (prime_vertical + altitude_m) * cos_lat * sin_lon,
            (prime_vertical * (1.0 - e2) + altitude_m) * sin_lat);
    }
};

} // namespace gnc::plugins::environment
