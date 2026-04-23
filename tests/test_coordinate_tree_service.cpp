#include "test_support.hpp"

#include "gnc/core/component_registry.hpp"
#include "gnc/environment/interfaces/i_earth.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_truth_view.hpp"
#include "gnc/forms/local_spherical_3dof/types.hpp"
#include "gnc/services/coordinate_tree/components/coordinate_tree_builder.hpp"
#include "gnc/services/coordinate_tree/components/coordinate_tree_service.hpp"
#include "gnc/services/coordinate_tree/internal/coordinate_tree_build_context.hpp"
#include "gnc/services/coordinate_tree/internal/coordinate_tree_spec_registry.hpp"
#include "gnc/services/coordinate_tree/specs/register_builtin_specs.hpp"

#include <exception>
#include <iostream>
#include <memory>
#include <string>

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

class FixedTruthView final : public gnc::core::ComponentBase,
                             public gnc::forms::local_spherical_3dof::ITruthView {
public:
    FixedTruthView() : ComponentBase("FixedTruthView") {
        truth_.velocity_launch_mps = gnc::math::Vector3::UnitX();
    }

    void update(double) override {}

    const gnc::forms::local_spherical_3dof::Truth& getLocalSpherical3DoFTruth() const override {
        return truth_;
    }

private:
    gnc::forms::local_spherical_3dof::Truth truth_{};
};

gnc::core::ConfigNode makeEmptyConfig() {
    return test_support::object({
        test_support::field("spec", test_support::string("empty")),
    });
}

gnc::core::ConfigNode makeLaunchTrackConfig() {
    using namespace test_support;
    return object({
        field("spec", string("local_spherical_3dof.launch_track")),
        field("bindings",
              object({
                  field("earth", object({field("name", string("env.earth"))})),
                  field("truth", object({field("name", string("truth"))})),
              })),
        field("launch",
              object({
                  field("latitude_rad", number(0.5235987755982988)),
                  field("longitude_rad", number(1.9198621771937625)),
                  field("azimuth_rad", number(1.5707963267948966)),
                  field("launch_time_s", number(0.0)),
                  field("earth_rotation_angle_rad", number(0.0)),
              })),
    });
}

std::shared_ptr<gnc::services::coordinate_tree::CoordinateTreeService> buildService(
    const gnc::services::coordinate_tree::internal::CoordinateTreeSpecRegistry& registry,
    const std::string& spec_id,
    gnc::core::ComponentRegistry& component_registry,
    const gnc::core::ConfigNode& config,
    const std::string& scope) {
    const auto* spec = registry.findSpec(spec_id);
    test_support::require(spec != nullptr,
                          "Requested coordinate-tree spec was not registered: " + spec_id);

    auto service = std::make_shared<gnc::services::coordinate_tree::CoordinateTreeService>();
    service->beginBuild();
    gnc::services::coordinate_tree::internal::CoordinateTreeBuildContext context(
        component_registry, scope, config);
    gnc::services::coordinate_tree::CoordinateTreeBuilder builder;
    spec->build(builder, context);
    service->loadBuiltTree(builder.seal());
    return service;
}

} // namespace

int main() {
    try {
        gnc::services::coordinate_tree::CoordinateTreeService unready_service;
        bool unready_failed = false;
        try {
            (void)unready_service.hasFrame("I");
        } catch (const std::exception& ex) {
            unready_failed =
                std::string(ex.what()).find("service wiring has not completed") !=
                std::string::npos;
        }
        test_support::require(
            unready_failed,
            "CoordinateTreeService should reject queries before finalize/load completes.");

        gnc::services::coordinate_tree::internal::CoordinateTreeSpecRegistry registry;
        gnc::services::coordinate_tree::specs::registerBuiltinCoordinateTreeSpecs(registry);

        gnc::core::ComponentRegistry empty_registry;
        const auto empty_config = makeEmptyConfig();
        auto empty_service =
            buildService(registry, "empty", empty_registry, empty_config, "vehicle");
        test_support::require(empty_service->hasFrame("I"),
                              "Empty coordinate-tree spec should build the root frame.");
        test_support::require(!empty_service->hasFrame("K"),
                              "Empty coordinate-tree spec should not add extra frames.");
        test_support::requireVectorNear(
            empty_service->transform(gnc::math::Vector3::UnitX(), "I", "I", 0.0),
            gnc::math::Vector3::UnitX(),
            1e-9,
            "Empty coordinate-tree spec should preserve identity transforms.");

        gnc::core::ComponentRegistry component_registry;
        component_registry.add<FixedEarth, gnc::environment::IEarth>(
            "env.earth", std::make_unique<FixedEarth>());
        component_registry.add<FixedTruthView, gnc::forms::local_spherical_3dof::ITruthView>(
            "vehicle.truth", std::make_unique<FixedTruthView>());

        const auto launch_track_config = makeLaunchTrackConfig();
        auto launch_track_service = buildService(
            registry,
            "local_spherical_3dof.launch_track",
            component_registry,
            launch_track_config,
            "vehicle");
        for (const char* frame_id : {"I", "E", "N", "L", "LI", "K"}) {
            test_support::require(
                launch_track_service->hasFrame(frame_id),
                std::string("Launch-track coordinate-tree spec is missing frame: ") +
                    frame_id);
        }
        test_support::require(launch_track_service->hasEdge("K", "L"),
                              "Launch-track coordinate-tree spec is missing K -> L.");
        test_support::requireVectorNear(
            launch_track_service->transform(gnc::math::Vector3::UnitX(), "K", "L", 0.0),
            gnc::math::Vector3::UnitX(),
            1e-9,
            "K -> L should align with the launch-frame velocity when truth velocity is UnitX.");

        std::cout << "coordinate tree service checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
