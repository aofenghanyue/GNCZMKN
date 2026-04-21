#include "test_support.hpp"

#include "gnc/bootstrap/install_builtin_services.hpp"
#include "gnc/core/component_registry.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/core/service_context.hpp"
#include "gnc/environment/interfaces/i_earth.hpp"
#include "gnc/services/soviet_coord/interfaces/i_coord_service.hpp"
#include "gnc/services/soviet_coord/interfaces/i_velocity_direction_provider.hpp"

#include <exception>
#include <iostream>
#include <memory>
#include <vector>

namespace {

class FixedEarth final : public gnc::core::ComponentBase,
                         public gnc::environment::IEarth {
public:
    FixedEarth() : ComponentBase("FixedEarth") {}
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

class FixedVelocityDirection final : public gnc::core::ComponentBase,
                                     public gnc::services::soviet_coord::IVelocityDirectionProvider {
public:
    FixedVelocityDirection() : ComponentBase("FixedVelocityDirection") {}
    void update(double) override {}
    gnc::math::Vector3 getVelocityInLaunchFrame() const override {
        return gnc::math::Vector3::UnitX();
    }
};

gnc::core::ConfigNode makeServiceConfig() {
    using namespace test_support;
    return object({
        field("launch",
              object({
                  field("latitude_rad", number(0.5235987755982988)),
                  field("longitude_rad", number(1.9198621771937625)),
                  field("azimuth_rad", number(1.5707963267948966)),
                  field("launch_time_s", number(0.0)),
                  field("earth_rotation_angle_rad", number(0.0)),
              })),
        field("bindings",
              object({
                  field("earth", object({field("name", string("earth"))})),
                  field("velocity_direction",
                        object({field("name", string("velocity"))})),
              })),
    });
}

} // namespace

int main() {
    try {
        gnc::core::ComponentRegistry registry;
        registry.add<FixedEarth, gnc::environment::IEarth>(
            "earth", std::make_unique<FixedEarth>());
        registry.add<FixedVelocityDirection, gnc::services::soviet_coord::IVelocityDirectionProvider>(
            "velocity", std::make_unique<FixedVelocityDirection>());

        gnc::core::ServiceContext services;
        std::vector<gnc::core::DeferredRegistryAction> deferred_actions;
        const auto config = makeServiceConfig();

        gnc::bootstrap::installBuiltinService(
            "soviet_coord", config, services, "", deferred_actions);
        test_support::require(services.get<gnc::services::soviet_coord::ICoordService>() != nullptr,
                              "Service installer did not register ICoordService.");

        for (const auto& action : deferred_actions) {
            action(registry);
        }

        auto* service = services.get<gnc::services::soviet_coord::ICoordService>();
        test_support::require(service->hasFrame("I"), "Inertial coordinate system is missing.");
        test_support::require(service->hasFrame("K"), "Track coordinate system was not installed.");
        test_support::require(!service->hasFrame("B"),
                              "Body coordinate system should not exist without an attitude provider.");
        test_support::require(service->hasEdge("K", "L"),
                              "Track-to-launch edge was not constructed.");

        const auto transformed =
            service->transform(gnc::math::Vector3::UnitX(), "K", "L", 0.0);
        test_support::requireVectorNear(
            transformed,
            gnc::math::Vector3::UnitX(),
            1e-9,
            "Track frame should align with launch frame for UnitX velocity.");

        std::cout << "soviet_coord service checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
