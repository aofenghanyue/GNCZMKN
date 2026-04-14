#pragma once

#include "gnc/core/config_manager.hpp"
#include "gnc/core/plugin_registry.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/plugins/soviet_coord/components/soviet_coord_service.hpp"

#include <memory>
#include <stdexcept>

namespace gnc::plugins::soviet_coord {

class SovietCoordPlugin final : public gnc::core::Plugin {
public:
    const char* name() const override { return "soviet_coord"; }
    gnc::core::PluginLayer layer() const override { return gnc::core::PluginLayer::System; }

    std::vector<std::string> dependencies() const override {
        return {"environment"};
    }

    void install(gnc::core::PluginRegistry& registry) const override {
        registry.registerServiceInstaller(
            "soviet_coord",
            [](const gnc::core::PluginRegistry::ServiceInstallRequest& request) {
                SovietCoordLaunchConfig launch_config;
                SovietCoordBindings bindings;

                const auto& launch = request.config["launch"];
                if (launch.isNull()) {
                    throw std::runtime_error("services.soviet_coord.launch is required.");
                }

                launch_config.latitude_rad = launch["latitude_rad"].asDouble(0.0);
                launch_config.longitude_rad = launch["longitude_rad"].asDouble(0.0);
                launch_config.azimuth_rad = launch["azimuth_rad"].asDouble(0.0);
                launch_config.launch_time_s = launch["launch_time_s"].asDouble(0.0);
                launch_config.earth_rotation_angle_rad =
                    launch["earth_rotation_angle_rad"].asDouble(0.0);

                const auto& binding_config = request.config["bindings"];
                if (binding_config.isNull()) {
                    throw std::runtime_error("services.soviet_coord.bindings is required.");
                }

                bindings.earth = binding_config["earth"]["name"].asString();
                if (bindings.earth.empty()) {
                    throw std::runtime_error(
                        "services.soviet_coord.bindings.earth.name is required.");
                }
                bindings.velocity_direction =
                    binding_config["velocity_direction"]["name"].asString();
                bindings.body_attitude =
                    binding_config["body_attitude"]["name"].asString();
                bindings.body_airspeed =
                    binding_config["body_airspeed"]["name"].asString();

                auto service = std::make_shared<SovietCoordService>(launch_config, bindings);
                request.services.registerService<ISovietCoordService>(service);

                request.deferred_actions.push_back(
                    [service, registry_scope = request.registry_scope](
                        gnc::core::ComponentRegistry& component_registry) {
                        gnc::core::ScopedRegistry scoped(registry_scope,
                                                         component_registry,
                                                         "soviet_coord_service");
                        service->bindProviders(scoped);
                    });
            });
    }
};

GNC_REGISTER_PLUGIN(SovietCoordPlugin)

} // namespace gnc::plugins::soviet_coord
