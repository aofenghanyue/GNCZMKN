#pragma once

#include "gnc/environment/interfaces/i_earth.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_truth_view.hpp"
#include "gnc/services/coordinate_tree/components/coordinate_tree_builder.hpp"
#include "gnc/services/coordinate_tree/interfaces/i_coordinate_tree_spec.hpp"
#include "gnc/services/coordinate_tree/internal/coordinate_tree_build_context.hpp"
#include "gnc/services/coordinate_tree/specs/internal/local_spherical_launch_track_math.hpp"

#include <string>
#include <string_view>
#include <stdexcept>

namespace gnc::services::coordinate_tree::specs {

class LocalSpherical3DoFLaunchTrackSpec final : public ICoordinateTreeSpec {
public:
    std::string_view id() const override {
        return "local_spherical_3dof.launch_track";
    }

    void build(CoordinateTreeBuilder& builder,
               const gnc::services::coordinate_tree::internal::CoordinateTreeBuildContext&
                   context) const override {
        const auto& config = context.config();
        const auto& bindings = requireObject(config, "bindings", "coordinate_tree.bindings");
        const auto& launch = requireObject(config, "launch", "coordinate_tree.launch");
        const auto& earth_binding =
            requireObject(bindings, "earth", "coordinate_tree.bindings.earth");
        const auto& truth_binding =
            requireObject(bindings, "truth", "coordinate_tree.bindings.truth");

        const std::string earth_name =
            requireString(earth_binding, "name", "coordinate_tree.bindings.earth.name");
        const std::string truth_name =
            requireString(truth_binding, "name", "coordinate_tree.bindings.truth.name");

        const double latitude_rad =
            requireNumber(launch, "latitude_rad", "coordinate_tree.launch.latitude_rad");
        const double longitude_rad =
            requireNumber(launch, "longitude_rad", "coordinate_tree.launch.longitude_rad");
        const double azimuth_rad =
            requireNumber(launch, "azimuth_rad", "coordinate_tree.launch.azimuth_rad");
        const double launch_time_s =
            requireNumber(launch, "launch_time_s", "coordinate_tree.launch.launch_time_s");
        const double earth_rotation_angle_rad = requireNumber(
            launch,
            "earth_rotation_angle_rad",
            "coordinate_tree.launch.earth_rotation_angle_rad");

        auto* earth =
            context.registry().requireByName<gnc::environment::IEarth>(earth_name);
        auto* truth_view = context.registry().requireByName<
            gnc::forms::local_spherical_3dof::ITruthView>(truth_name);

        builder.setRoot("I");
        builder.addFrame("E");
        builder.addFrame("N");
        builder.addFrame("L");
        builder.addFrame("LI");
        builder.addFrame("K");

        builder.addDynamicEdge(
            "E",
            "I",
            [earth, earth_rotation_angle_rad](double time) {
                return specs::internal::ecefToInertialRotation(
                    earth_rotation_angle_rad + earth->getRotationRate() * time);
            });
        builder.addStaticEdge(
            "N",
            "E",
            specs::internal::localGeographicToEarthFixedRotation(latitude_rad,
                                                                 longitude_rad));
        builder.addStaticEdge("L",
                              "N",
                              specs::internal::launchToLocalGeographicRotation(azimuth_rad));
        builder.addDynamicEdge(
            "LI",
            "L",
            [earth, launch_time_s](double time) {
                return specs::internal::launchInertialToLaunchRotation(
                    launch_time_s, time, earth->getRotationRate());
            });
        builder.addDynamicEdge(
            "K",
            "L",
            [truth_view](double) {
                return specs::internal::trackToLaunchRotation(
                    truth_view->getLocalSpherical3DoFTruth().velocity_launch_mps);
            });
    }

private:
    static const gnc::core::ConfigNode& requireObject(const gnc::core::ConfigNode& node,
                                                      const char* key,
                                                      const char* path) {
        const auto& child = node[key];
        if (!child.isObject()) {
            throw std::runtime_error(std::string(path) +
                                     " is required and must be an object.");
        }
        return child;
    }

    static std::string requireString(const gnc::core::ConfigNode& node,
                                     const char* key,
                                     const char* path) {
        const auto& child = node[key];
        if (!child.isString() || child.asString().empty()) {
            throw std::runtime_error(std::string(path) +
                                     " is required and must be a non-empty string.");
        }
        return child.asString();
    }

    static double requireNumber(const gnc::core::ConfigNode& node,
                                const char* key,
                                const char* path) {
        const auto& child = node[key];
        if (!child.isNumber()) {
            throw std::runtime_error(std::string(path) +
                                     " is required and must be a number.");
        }
        return child.asDouble();
    }
};

} // namespace gnc::services::coordinate_tree::specs
