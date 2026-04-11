#include "test_support.hpp"

#include "gnc/plugins/environment/components/spherical_gravity.hpp"
#include "gnc/plugins/environment/components/standard_atmosphere.hpp"
#include "gnc/plugins/environment/components/wgs84_earth.hpp"

#include <exception>
#include <iostream>

int main() {
    try {
        gnc::plugins::environment::Wgs84Earth earth;
        gnc::plugins::environment::StandardAtmosphere atmosphere;
        gnc::plugins::environment::SphericalGravity gravity;

        test_support::requireNear(earth.getEquatorialRadius(),
                                  6378137.0,
                                  1e-6,
                                  "Unexpected WGS84 equatorial radius.");

        const auto ecef_origin = earth.geodeticToEcef(0.0, 0.0, 0.0);
        test_support::requireVectorNear(
            ecef_origin,
            gnc::math::Vector3(earth.getEquatorialRadius(), 0.0, 0.0),
            1e-6,
            "Equatorial geodetic-to-ECEF conversion drifted.");

        const double rho0 = atmosphere.getDensity(0.0);
        const double rho10 = atmosphere.getDensity(10000.0);
        test_support::require(rho0 > rho10,
                              "Atmospheric density should decrease with altitude.");

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
