#include "test_support.hpp"

#include "gnc/core/component_registry.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/plugins/aero/interfaces/i_aero_model.hpp"
#include "gnc/plugins/environment/interfaces/i_atmosphere.hpp"
#include "gnc/plugins/environment/interfaces/i_earth.hpp"
#include "gnc/plugins/environment/interfaces/i_gravity.hpp"
#include "gnc/plugins/state_3dof/components/point_mass_cartesian.hpp"
#include "gnc/plugins/state_3dof/components/point_mass_spherical.hpp"
#include "gnc/plugins/state_3dof/interfaces/i_flight_command_provider_3dof.hpp"

#include <exception>
#include <iostream>
#include <memory>

namespace {

class ConstantAtmosphere final : public gnc::core::ComponentBase,
                                 public gnc::plugins::environment::IAtmosphere {
public:
    ConstantAtmosphere() : ComponentBase("ConstantAtmosphere") {}
    void update(double) override {}
    double getDensity(double) const override { return 0.02; }
    double getPressure(double) const override { return 1200.0; }
    double getTemperature(double) const override { return 240.0; }
    double getSpeedOfSound(double) const override { return 300.0; }
};

class ConstantGravity final : public gnc::core::ComponentBase,
                              public gnc::plugins::environment::IGravity {
public:
    ConstantGravity() : ComponentBase("ConstantGravity") {}
    void update(double) override {}
    double getSeaLevelGravity() const override { return 9.81; }
    double getGravityMagnitude(double) const override { return 9.81; }
    gnc::math::Vector3 getGravityVector(const gnc::math::Vector3& position_ecef) const override {
        return -9.81 * position_ecef.normalized();
    }
};

class ConstantEarth final : public gnc::core::ComponentBase,
                            public gnc::plugins::environment::IEarth {
public:
    ConstantEarth() : ComponentBase("ConstantEarth") {}
    void update(double) override {}
    double getEquatorialRadius() const override { return 6378137.0; }
    double getFlattening() const override { return 1.0 / 298.257223563; }
    double getRotationRate() const override { return 7.292115e-5; }
    gnc::math::Vector3 geodeticToEcef(double latitude_rad,
                                      double longitude_rad,
                                      double altitude_m) const override {
        const double radius = getEquatorialRadius() + altitude_m;
        return gnc::math::Vector3(radius * std::cos(latitude_rad) * std::cos(longitude_rad),
                                  radius * std::cos(latitude_rad) * std::sin(longitude_rad),
                                  radius * std::sin(latitude_rad));
    }
};

class ConstantAeroModel final : public gnc::core::ComponentBase,
                                public gnc::plugins::aero::IAeroModel {
public:
    ConstantAeroModel() : ComponentBase("ConstantAeroModel") {}
    void update(double) override {}
    gnc::plugins::aero::AeroCoefficients computeCoefficients(double,
                                                             double,
                                                             double) const override {
        return {0.35, 0.18};
    }
    double getReferenceArea() const override { return 0.48; }
    double getReferenceLength() const override { return 2.5; }
};

class ConstantGuidance final : public gnc::core::ComponentBase,
                               public gnc::plugins::state_3dof::IFlightCommandProvider3DOF {
public:
    ConstantGuidance() : ComponentBase("ConstantGuidance") {}
    void update(double) override {}
    const gnc::plugins::state_3dof::FlightCommand3DOF& getFlightCommand() const override {
        return command_;
    }
    bool isActive() const override { return true; }

private:
    gnc::plugins::state_3dof::FlightCommand3DOF command_{0.08, 0.0, 0.0};
};

gnc::core::ConfigNode makeCartesianConfig() {
    using namespace test_support;
    return object({
        field("initial_position", array({number(0.0), number(0.0), number(1000.0)})),
        field("initial_velocity", array({number(250.0), number(0.0), number(40.0)})),
        field("constant_acceleration", array({number(0.0), number(0.0), number(-9.81)})),
        field("mass_kg", number(50.0)),
    });
}

gnc::core::ConfigNode makeSphericalConfig() {
    using namespace test_support;
    return object({
        field("launch_azimuth_rad", number(1.5707963267948966)),
        field("mass_kg", number(900.0)),
        field("initial_state",
              object({
                  field("longitude_rad", number(1.9198621771937625)),
                  field("latitude_rad", number(0.5235987755982988)),
                  field("altitude_m", number(60000.0)),
                  field("speed_mps", number(3200.0)),
                  field("flight_path_angle_rad", number(-0.10)),
                  field("heading_angle_rad", number(1.5707963267948966)),
              })),
    });
}

} // namespace

int main() {
    try {
        gnc::plugins::state_3dof::PointMassCartesian cartesian;
        cartesian.configure(makeCartesianConfig());

        Eigen::VectorXd cartesian_dx;
        cartesian.computeDerivatives(0.0, cartesian.getState(), cartesian_dx);
        test_support::require(cartesian.getStateLayout().dimension() == 6,
                              "Cartesian 3DOF state dimension must remain 6.");
        test_support::requireNear(cartesian_dx[0], 250.0, 1e-9,
                                  "Cartesian position derivative no longer matches velocity.");
        test_support::requireNear(cartesian_dx[5], -9.81, 1e-9,
                                  "Cartesian acceleration derivative drifted.");
        test_support::requireNear(cartesian.getAltitude(), 1000.0, 1e-9,
                                  "Cartesian altitude accessor returned an unexpected value.");

        gnc::core::ComponentRegistry registry;
        registry.add<ConstantAtmosphere, gnc::plugins::environment::IAtmosphere>(
            "atmosphere", std::make_unique<ConstantAtmosphere>());
        registry.add<ConstantGravity, gnc::plugins::environment::IGravity>(
            "gravity", std::make_unique<ConstantGravity>());
        registry.add<ConstantEarth, gnc::plugins::environment::IEarth>(
            "earth", std::make_unique<ConstantEarth>());
        registry.add<ConstantAeroModel, gnc::plugins::aero::IAeroModel>(
            "aero", std::make_unique<ConstantAeroModel>());
        registry.add<ConstantGuidance, gnc::plugins::state_3dof::IFlightCommandProvider3DOF>(
            "guidance", std::make_unique<ConstantGuidance>());

        gnc::plugins::state_3dof::PointMassSpherical spherical;
        spherical.configure(makeSphericalConfig());
        gnc::core::ScopedRegistry scoped("", registry, "dynamics");
        spherical.injectDependencies(scoped);

        Eigen::VectorXd spherical_dx;
        spherical.computeDerivatives(0.0, spherical.getState(), spherical_dx);
        test_support::require(spherical.getStateLayout().dimension() == 6,
                              "Spherical 3DOF state dimension must remain 6.");
        test_support::require(spherical_dx[2] < 0.0,
                              "Descending initial condition should reduce altitude.");
        test_support::require(spherical.getVelocityInLaunchFrame().x() > 0.0,
                              "Launch-frame velocity should point downrange for the configured heading.");
        test_support::require(spherical.getPosition().norm() > 6.3e6,
                              "ECEF position magnitude is inconsistent with the configured altitude.");

        std::cout << "state_3dof plugin checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
