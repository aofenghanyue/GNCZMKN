#include "test_support.hpp"

#include "gnc/plugins/environment/components/spherical_earth.hpp"
#include "gnc/plugins/environment/components/spherical_gravity.hpp"
#include "gnc/plugins/environment/components/standard_atmosphere.hpp"
#include "gnc/plugins/environment/components/wgs84_earth.hpp"

#include <exception>
#include <iostream>

int main() {
    try {
        gnc::plugins::environment::Wgs84Earth wgs84_earth;
        gnc::plugins::environment::SphericalEarth spherical_earth;
        gnc::plugins::environment::StandardAtmosphere atmosphere;
        gnc::plugins::environment::SphericalGravity gravity;

        test_support::requireNear(wgs84_earth.getEquatorialRadius(),
                                  6378137.0,
                                  1e-6,
                                  "Unexpected WGS84 equatorial radius.");
        test_support::requireNear(spherical_earth.getFlattening(),
                                  0.0,
                                  1e-12,
                                  "Spherical Earth flattening must be zero.");

        const auto ecef_origin = wgs84_earth.geodeticToEcef(0.0, 0.0, 0.0);
        test_support::requireVectorNear(
            ecef_origin,
            gnc::math::Vector3(wgs84_earth.getEquatorialRadius(), 0.0, 0.0),
            1e-6,
            "Equatorial geodetic-to-ECEF conversion drifted.");

        const auto sea_level = atmosphere.sample(0.0);
        const auto ten_km = atmosphere.sample(10000.0);
        test_support::require(sea_level.density_kg_per_m3 > ten_km.density_kg_per_m3,
                              "Atmospheric density should decrease with altitude.");
        test_support::requireNear(atmosphere.getSeaLevelPressure(),
                                  sea_level.pressure_pa,
                                  1e-9,
                                  "Sea-level pressure accessor drifted from the sample table.");
        test_support::requireNear(atmosphere.getSeaLevelTemperature(),
                                  sea_level.temperature_k,
                                  1e-9,
                                  "Sea-level temperature accessor drifted from the sample table.");

        const auto gravity_vector =
            gravity.getGravityVector(gnc::math::Vector3(6371000.0, 0.0, 0.0));
        test_support::requireNear(gravity.getGravityMagnitude(0.0),
                                  9.80665,
                                  1e-5,
                                  "Sea-level gravity changed unexpectedly.");
        test_support::requireVectorNear(
            gravity_vector,
            gnc::math::Vector3(-9.80665, 0.0, 0.0),
            1e-4,
            "Gravity vector direction is inconsistent with spherical gravity.");

        std::cout << "environment plugin checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
