#include "gnc/core/component_factory.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/core/simulation_builder.hpp"
#include "gnc/interfaces/environment/i_earth_model.hpp"
#include "gnc/interfaces/state/i_attitude_provider.hpp"
#include "gnc/interfaces/state/i_position_provider.hpp"
#include "gnc/interfaces/state/i_velocity_provider.hpp"
#include "gnc/libraries/coord/rotations.hpp"
#include "gnc/services/coordinate/coordinate_service.hpp"
#include "gnc/services/coordinate/soviet_coordinate_system.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

namespace soviet = gnc::services::coordinate::soviet;

constexpr double kTol = 1e-9;

bool matrixClose(const gnc::math::Matrix3& lhs,
                 const gnc::math::Matrix3& rhs,
                 double tol = kTol) {
    return (lhs - rhs).cwiseAbs().maxCoeff() <= tol;
}

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

gnc::Vector3d readVector3(const gnc::core::ConfigNode& node) {
    return {
        node[0].asDouble(0.0),
        node[1].asDouble(0.0),
        node[2].asDouble(0.0)
    };
}

gnc::Quaterniond readQuaternion(const gnc::core::ConfigNode& node) {
    return {
        node[0].asDouble(1.0),
        node[1].asDouble(0.0),
        node[2].asDouble(0.0),
        node[3].asDouble(0.0)
    };
}

class TestEarthComponent : public gnc::core::ComponentBase,
                           public gnc::interfaces::IEarthModel {
public:
    TestEarthComponent() : ComponentBase("TestEarthComponent") {}

    void update(double) override {}

    double getRadius() const override { return 6378137.0; }
    double getRotationRate() const override { return 0.01; }
    double getFlattening() const override { return 0.0; }

    gnc::Vector3d geodeticToEcef(double lat, double lon, double alt) const override {
        const double r = getRadius() + alt;
        return {
            r * std::cos(lat) * std::cos(lon),
            r * std::cos(lat) * std::sin(lon),
            r * std::sin(lat)
        };
    }

    void ecefToGeodetic(const gnc::Vector3d& ecef,
                        double& lat,
                        double& lon,
                        double& alt) const override {
        lon = std::atan2(ecef.y, ecef.x);
        const double planar = std::sqrt(ecef.x * ecef.x + ecef.y * ecef.y);
        lat = std::atan2(ecef.z, planar);
        alt = std::sqrt(ecef.x * ecef.x + ecef.y * ecef.y + ecef.z * ecef.z) - getRadius();
    }
};

class TestPositionProvider : public gnc::core::ComponentBase,
                             public gnc::interfaces::IPositionProvider {
public:
    TestPositionProvider() : ComponentBase("TestPositionProvider") {}

    void configure(const gnc::core::ConfigNode& config) override {
        position_ = readVector3(config["position"]);
    }

    void update(double) override {}

    gnc::Vector3d getPosition() const override { return position_; }

private:
    gnc::Vector3d position_ = gnc::Vector3d::Zero();
};

class TestAttitudeProvider : public gnc::core::ComponentBase,
                             public gnc::interfaces::IAttitudeProvider {
public:
    TestAttitudeProvider() : ComponentBase("TestAttitudeProvider") {}

    void configure(const gnc::core::ConfigNode& config) override {
        attitude_ = readQuaternion(config["attitude"]);
    }

    void update(double) override {}

    gnc::Quaterniond getAttitude() const override { return attitude_; }

private:
    gnc::Quaterniond attitude_ = gnc::Quaterniond::Identity();
};

class TestVelocityProvider : public gnc::core::ComponentBase,
                             public gnc::interfaces::IVelocityProvider {
public:
    TestVelocityProvider() : ComponentBase("TestVelocityProvider") {}

    void configure(const gnc::core::ConfigNode& config) override {
        velocity_ = readVector3(config["velocity"]);
    }

    void update(double) override {}

    gnc::Vector3d getVelocity() const override { return velocity_; }

private:
    gnc::Vector3d velocity_ = gnc::Vector3d::Zero();
};

GNC_REGISTER_COMPONENT(TestEarthComponent, gnc::interfaces::IEarthModel)
GNC_REGISTER_COMPONENT(TestPositionProvider, gnc::interfaces::IPositionProvider)
GNC_REGISTER_COMPONENT(TestAttitudeProvider, gnc::interfaces::IAttitudeProvider)
GNC_REGISTER_COMPONENT(TestVelocityProvider, gnc::interfaces::IVelocityProvider)

void expectBuildFailure(const std::string& config, const std::string& expected_fragment) {
    gnc::core::SimulationBuilder builder;
    require(builder.loadConfigString(config), "Failed to load failure-test config.");

    bool failed = false;
    try {
        builder.build();
    } catch (const std::exception& e) {
        failed = true;
        const std::string message = e.what();
        const bool matched =
            expected_fragment.empty() ||
            message.find(expected_fragment) != std::string::npos ||
            message.find("Simulation build failed with") != std::string::npos;
        require(matched, "Build failed for the wrong reason: " + message);
    }

    require(failed, "Expected the build to fail, but it succeeded.");
}

void testGenericRotationTree() {
    gnc::services::CoordinateService service;
    require(service.hasFrame(gnc::coord::FrameId::ECI), "CoordinateService should initialize the root frame.");

    int constant_calls = 0;
    service.registerTransform(gnc::coord::FrameId::ECEF, gnc::coord::FrameId::ECI, [&]() {
        ++constant_calls;
        return gnc::coord::rotationAboutZ(0.2);
    });

    int timed_calls = 0;
    service.registerTransform(gnc::coord::FrameId::NUE, gnc::coord::FrameId::ECEF, [&](double time) {
        ++timed_calls;
        return gnc::coord::rotationAboutY(0.3 + 0.1 * time);
    });

    service.registerTransform(gnc::coord::FrameId::LAUNCH, gnc::coord::FrameId::NUE, []() {
        return gnc::coord::rotationAboutX(0.25);
    });

    const auto launch_to_eci_expected =
        gnc::coord::rotationAboutZ(0.2) *
        gnc::coord::rotationAboutY(0.3) *
        gnc::coord::rotationAboutX(0.25);
    require(matrixClose(service.getRotation(gnc::coord::FrameId::LAUNCH,
                                            gnc::coord::FrameId::ECI,
                                            0.0),
                        launch_to_eci_expected),
            "Coordinate tree path composition is incorrect.");
    require(matrixClose(service.getRotation(gnc::coord::FrameId::ECI,
                                            gnc::coord::FrameId::LAUNCH,
                                            0.0),
                        launch_to_eci_expected.transpose()),
            "Inverse path composition is incorrect.");

    require(matrixClose(service.getRotation(gnc::coord::FrameId::ECEF,
                                            gnc::coord::FrameId::ECI,
                                            0.0),
                        gnc::coord::rotationAboutZ(0.2)),
            "Direct child-to-parent rotation is incorrect.");
    require(matrixClose(service.getRotation(gnc::coord::FrameId::ECEF,
                                            gnc::coord::FrameId::ECI,
                                            10.0),
                        gnc::coord::rotationAboutZ(0.2)),
            "Constant rotation should not depend on time.");
    require(constant_calls == 1, "Constant rotation should be cached.");

    const auto same_time_first = service.getRotation(gnc::coord::FrameId::NUE,
                                                     gnc::coord::FrameId::ECEF,
                                                     2.0);
    const auto same_time_second = service.getRotation(gnc::coord::FrameId::NUE,
                                                      gnc::coord::FrameId::ECEF,
                                                      2.0);
    require(matrixClose(same_time_first, same_time_second),
            "Timed transform should be stable for the same query time.");
    require(timed_calls == 2, "Timed transform should be cached for repeated query times.");

    const auto different_time = service.getRotation(gnc::coord::FrameId::NUE,
                                                    gnc::coord::FrameId::ECEF,
                                                    3.0);
    require(!matrixClose(same_time_first, different_time),
            "Timed transform should refresh for a new query time.");
    require(timed_calls == 3, "Timed transform should be recomputed for a new query time.");

    bool cycle_guarded = false;
    try {
        service.registerTransform(gnc::coord::FrameId::ECI, gnc::coord::FrameId::LAUNCH, []() {
            return gnc::math::Matrix3::Identity();
        });
    } catch (const std::exception&) {
        cycle_guarded = true;
    }
    require(cycle_guarded, "The root frame must not accept a parent.");
}

void testDefaultSovietSchemeInstallation() {
    const std::string config = R"json(
{
  "simulation": { "dt": 0.1, "duration": 1.0 },
  "services": {
    "coordinate": {
      "enabled": true,
      "launch": {
        "latitude_rad": 0.2,
        "longitude_rad": 0.1,
        "azimuth_rad": 0.3
      },
      "bindings": {
        "earth": { "name": "earth" }
      }
    }
  },
  "components": [
    { "type": "TestEarthComponent", "name": "earth" }
  ]
}
)json";

    gnc::core::SimulationBuilder builder;
    require(builder.loadConfigString(config), "Failed to load default-scheme config.");
    builder.build();

    auto* service = builder.getGlobalServices().get<gnc::services::CoordinateService>();
    require(service != nullptr, "CoordinateService should be registered.");
    require(service->hasFrame(gnc::coord::FrameId::ECI), "ECI root should exist.");
    require(service->hasFrame(gnc::coord::FrameId::ECEF), "ECEF should exist.");
    require(service->hasFrame(gnc::coord::FrameId::LAUNCH), "LAUNCH should exist.");
    require(service->hasFrame(gnc::coord::FrameId::LAUNCH_INERTIAL), "LAUNCH_INERTIAL should exist.");
    require(!service->hasFrame(gnc::coord::FrameId::NUE), "NUE should stay optional.");
}

void testSingleFlightDynamicBindingsDefaultSubject() {
    const std::string config = R"json(
{
  "simulation": { "dt": 0.1, "duration": 1.0 },
  "services": {
    "coordinate": {
      "enabled": true,
      "scheme": "soviet",
      "launch": {
        "latitude_rad": 0.45,
        "longitude_rad": 0.55,
        "azimuth_rad": 0.3,
        "t0": 0.25,
        "earth_rotation_angle_rad": 0.1
      },
      "bindings": {
        "earth": { "name": "earth" },
        "local_geographic": { "name": "local_geo" },
        "body_attitude": { "name": "attitude" },
        "track_motion": { "name": "ground_velocity" },
        "wind_velocity": { "name": "air_velocity" }
      }
    }
  },
  "components": [
    { "type": "TestEarthComponent", "name": "earth" },
    {
      "type": "TestPositionProvider",
      "name": "local_geo",
      "config": { "position": [0.4, 0.6, 1000.0] }
    },
    {
      "type": "TestAttitudeProvider",
      "name": "attitude",
      "config": { "attitude": [0.9238795325, 0.0, 0.3826834324, 0.0] }
    },
    {
      "type": "TestVelocityProvider",
      "name": "ground_velocity",
      "config": { "velocity": [120.0, 15.0, -30.0] }
    },
    {
      "type": "TestVelocityProvider",
      "name": "air_velocity",
      "config": { "velocity": [200.0, -20.0, 30.0] }
    }
  ]
}
)json";

    gnc::core::SimulationBuilder builder;
    require(builder.loadConfigString(config), "Failed to load dynamic Soviet config.");
    builder.build();

    auto* service = builder.getGlobalServices().get<gnc::services::CoordinateService>();
    require(service != nullptr, "CoordinateService should be registered.");
    require(service->hasFrame(gnc::coord::FrameId::NUE), "Dynamic NUE should be installed.");
    require(service->hasFrame(gnc::coord::FrameId::BODY), "BODY should be installed.");
    require(service->hasFrame(gnc::coord::FrameId::TRACK), "TRACK should be installed.");
    require(service->hasFrame(gnc::coord::FrameId::WIND), "WIND should be installed.");

    require(matrixClose(service->getRotation(gnc::coord::FrameId::ECEF,
                                             gnc::coord::FrameId::NUE,
                                             0.0),
                        soviet::earthFixedToLocalNueRotation(0.4, 0.6)),
            "ECEF->NUE rotation mismatch.");

    const auto expected_launch_to_ecef = soviet::launchToEarthFixedRotation(0.45, 0.55, 0.3);
    require(matrixClose(service->getRotation(gnc::coord::FrameId::LAUNCH,
                                             gnc::coord::FrameId::ECEF,
                                             0.0),
                        expected_launch_to_ecef),
            "LAUNCH->ECEF rotation mismatch.");

    const auto expected_li_to_l = soviet::launchInertialToLaunchRotation(
        1.0 - 0.25,
        soviet::earthFixedToLaunchRotation(0.45, 0.55, 0.3),
        0.01);
    require(matrixClose(service->getRotation(gnc::coord::FrameId::LAUNCH_INERTIAL,
                                             gnc::coord::FrameId::LAUNCH,
                                             1.0),
                        expected_li_to_l),
            "LAUNCH_INERTIAL->LAUNCH rotation mismatch.");

    const auto attitude = gnc::math::Quaternion(0.9238795325, 0.0, 0.3826834324, 0.0);
    require(matrixClose(service->getRotation(gnc::coord::FrameId::BODY,
                                             gnc::coord::FrameId::LAUNCH,
                                             0.0),
                        soviet::bodyToReferenceRotation(attitude)),
            "BODY->LAUNCH rotation mismatch.");

    double chi_rad = 0.0;
    double theta_rad = 0.0;
    soviet::computeTrackAngles(gnc::math::Vector3(120.0, 15.0, -30.0), chi_rad, theta_rad);
    require(matrixClose(service->getRotation(gnc::coord::FrameId::TRACK,
                                             gnc::coord::FrameId::LAUNCH,
                                             0.0),
                        soviet::trackToReferenceRotation(chi_rad, theta_rad)),
            "TRACK->LAUNCH rotation mismatch.");

    double alpha_rad = 0.0;
    double beta_rad = 0.0;
    soviet::computeWindAngles(gnc::math::Vector3(200.0, -20.0, 30.0), alpha_rad, beta_rad);
    require(matrixClose(service->getRotation(gnc::coord::FrameId::WIND,
                                             gnc::coord::FrameId::BODY,
                                             0.0),
                        soviet::windToBodyRotation(alpha_rad, beta_rad)),
            "WIND->BODY rotation mismatch.");
}

void testVehicleScopedDynamicBindingsSurviveMove() {
    const std::string config = R"json(
{
  "simulation": { "dt": 0.1, "duration": 1.0 },
  "environment": {
    "components": [
      { "type": "TestEarthComponent", "name": "earth" }
    ]
  },
  "vehicles": [
    {
      "id": "chaser",
      "services": {
        "coordinate": {
          "enabled": true,
          "scheme": "soviet",
          "launch": {
            "latitude_rad": 0.2,
            "longitude_rad": 0.1,
            "azimuth_rad": 0.0
          },
          "bindings": {
            "earth": { "name": "env.earth" },
            "local_geographic": { "name": "local_geo" },
            "body_attitude": { "name": "attitude" },
            "track_motion": { "name": "ground_velocity" }
          }
        }
      },
      "components": [
        {
          "type": "TestPositionProvider",
          "name": "local_geo",
          "config": { "position": [0.2, 0.1, 0.0] }
        },
        {
          "type": "TestAttitudeProvider",
          "name": "attitude",
          "config": { "attitude": [1.0, 0.0, 0.0, 0.0] }
        },
        {
          "type": "TestVelocityProvider",
          "name": "ground_velocity",
          "config": { "velocity": [80.0, 5.0, 0.0] }
        }
      ]
    }
  ]
}
)json";

    gnc::core::SimulationBuilder builder;
    require(builder.loadConfigString(config), "Failed to load vehicle-scoped config.");
    builder.build();

    auto& vehicles = builder.getVehicles();
    require(vehicles.size() == 1, "Expected one flight vehicle.");
    auto* service = vehicles.front().services.get<gnc::services::CoordinateService>();
    require(service != nullptr, "Flight-vehicle scoped CoordinateService should be registered.");
    require(service->hasFrame(gnc::coord::FrameId::NUE), "Vehicle-scoped NUE should be installed.");
    require(service->hasFrame(gnc::coord::FrameId::BODY), "Vehicle-scoped BODY should be installed.");
    require(service->hasFrame(gnc::coord::FrameId::TRACK), "Vehicle-scoped TRACK should be installed.");
}

void testUnknownSchemeFails() {
    const std::string config = R"json(
{
  "simulation": { "dt": 0.1, "duration": 1.0 },
  "services": {
    "coordinate": {
      "enabled": true,
      "scheme": "unknown"
    }
  }
}
)json";

    expectBuildFailure(config, "Unknown coordinate service scheme");
}

void testGlobalDynamicBindingsRequireExplicitSubject() {
    const std::string config = R"json(
{
  "simulation": { "dt": 0.1, "duration": 1.0 },
  "global_services": {
    "coordinate": {
      "enabled": true,
      "scheme": "soviet",
      "launch": {
        "latitude_rad": 0.2,
        "longitude_rad": 0.1,
        "azimuth_rad": 0.0
      },
      "bindings": {
        "earth": { "name": "env.earth" },
        "local_geographic": { "name": "chaser.local_geo" }
      }
    }
  },
  "environment": {
    "components": [
      { "type": "TestEarthComponent", "name": "earth" }
    ]
  },
  "vehicles": [
    {
      "id": "chaser",
      "components": [
        {
          "type": "TestPositionProvider",
          "name": "local_geo",
          "config": { "position": [0.2, 0.1, 0.0] }
        }
      ]
    }
  ]
}
)json";

    expectBuildFailure(config, "bindings.subject");
}

void testEnvironmentDynamicBindingsRequireExplicitSubject() {
    const std::string config = R"json(
{
  "simulation": { "dt": 0.1, "duration": 1.0 },
  "environment": {
    "services": {
      "coordinate": {
        "enabled": true,
        "scheme": "soviet",
        "launch": {
          "latitude_rad": 0.2,
          "longitude_rad": 0.1,
          "azimuth_rad": 0.0
        },
        "bindings": {
          "earth": { "name": "earth" },
          "local_geographic": { "name": "local_geo" }
        }
      }
    },
    "components": [
      { "type": "TestEarthComponent", "name": "earth" },
      {
        "type": "TestPositionProvider",
        "name": "local_geo",
        "config": { "position": [0.2, 0.1, 0.0] }
      }
    ]
  }
}
)json";

    expectBuildFailure(config, "bindings.subject");
}

void testDeprecatedPositionRepresentationFails() {
    const std::string config = R"json(
{
  "simulation": { "dt": 0.1, "duration": 1.0 },
  "services": {
    "coordinate": {
      "enabled": true,
      "launch": {
        "latitude_rad": 0.2,
        "longitude_rad": 0.1,
        "azimuth_rad": 0.0
      },
      "bindings": {
        "earth": { "name": "earth" },
        "local_geographic": {
          "name": "local_geo",
          "position_repr": "geodetic"
        }
      }
    }
  },
  "components": [
    { "type": "TestEarthComponent", "name": "earth" },
    {
      "type": "TestPositionProvider",
      "name": "local_geo",
      "config": { "position": [0.2, 0.1, 0.0] }
    }
  ]
}
)json";

    expectBuildFailure(config, "unsupported field");
}

void testDeprecatedTrackFrameFieldFails() {
    const std::string config = R"json(
{
  "simulation": { "dt": 0.1, "duration": 1.0 },
  "services": {
    "coordinate": {
      "enabled": true,
      "launch": {
        "latitude_rad": 0.2,
        "longitude_rad": 0.1,
        "azimuth_rad": 0.0
      },
      "bindings": {
        "earth": { "name": "earth" },
        "track_motion": {
          "name": "ground_velocity",
          "frame": "LAUNCH"
        }
      }
    }
  },
  "components": [
    { "type": "TestEarthComponent", "name": "earth" },
    {
      "type": "TestVelocityProvider",
      "name": "ground_velocity",
      "config": { "velocity": [10.0, 0.0, 0.0] }
    }
  ]
}
)json";

    expectBuildFailure(config, "unsupported field");
}

bool customRelationEnabled(const soviet::SovietInstallContext&) {
    return true;
}

void installCustomRelation(gnc::services::CoordinateService& service,
                           const soviet::SovietRelationSpec&,
                           const soviet::SovietInstallContext&) {
    service.registerTransform(gnc::coord::FrameId::ECEF, gnc::coord::FrameId::ECI, []() {
        return gnc::math::Matrix3::Identity();
    });
}

void testCustomSovietRelationSpecInstallsThroughUnifiedHook() {
    gnc::core::ComponentRegistry registry;
    gnc::core::ScopedRegistry scoped("", registry, "coordinate_service");

    soviet::SovietInstallContext context;
    context.registry = &scoped;
    context.service_scope_name = "flight_vehicle";
    context.subject = "flight_vehicle";

    const soviet::SovietRelationSpec relation{
        static_cast<soviet::SovietRelationId>(999),
        gnc::coord::FrameId::ECEF,
        gnc::coord::FrameId::ECI,
        nullptr,
        "Test extension relation",
        "No inputs",
        customRelationEnabled,
        installCustomRelation
    };

    gnc::services::CoordinateService service;
    soviet::installSovietRelation(service, relation, context);
    require(matrixClose(service.getRotation(gnc::coord::FrameId::ECEF,
                                            gnc::coord::FrameId::ECI,
                                            0.0),
                        gnc::math::Matrix3::Identity()),
            "Custom Soviet relation spec should install through the unified hook.");
}

} // namespace

int main() {
    try {
        testGenericRotationTree();
        testDefaultSovietSchemeInstallation();
        testSingleFlightDynamicBindingsDefaultSubject();
        testVehicleScopedDynamicBindingsSurviveMove();
        testUnknownSchemeFails();
        testGlobalDynamicBindingsRequireExplicitSubject();
        testEnvironmentDynamicBindingsRequireExplicitSubject();
        testDeprecatedPositionRepresentationFails();
        testDeprecatedTrackFrameFieldFails();
        testCustomSovietRelationSpecInstallsThroughUnifiedHook();
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }

    std::cout << "Coordinate service tests passed\n";
    return 0;
}
