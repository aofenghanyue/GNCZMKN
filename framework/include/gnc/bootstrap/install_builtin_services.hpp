#pragma once

#include "gnc/common/string_utils.hpp"
#include "gnc/core/component_registry.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/core/deferred_registry_action.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/core/service_context.hpp"
#include "gnc/services/soviet_coord/components/soviet_coord_service.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace gnc::bootstrap {

inline std::vector<std::string> getBuiltinServiceNames() {
    return {"soviet_coord"};
}

inline void installBuiltinService(const std::string& service_name,
                                  const gnc::core::ConfigNode& config,
                                  gnc::core::ServiceContext& services,
                                  const std::string& registry_scope,
                                  std::vector<gnc::core::DeferredRegistryAction>& deferred_actions) {
    if (service_name != "soviet_coord") {
        const std::string suggestion =
            gnc::common::findClosestMatch(service_name, getBuiltinServiceNames());
        std::string message = "Unknown builtin service '" + service_name + "'.";
        if (!suggestion.empty()) {
            message += " Did you mean '" + suggestion + "'?";
        }
        throw std::runtime_error(message);
    }

    gnc::services::soviet_coord::SovietCoordLaunchConfig launch_config;
    gnc::services::soviet_coord::SovietCoordBindings bindings;

    const auto& launch = config["launch"];
    if (launch.isNull()) {
        throw std::runtime_error("services.soviet_coord.launch is required.");
    }

    launch_config.latitude_rad = launch["latitude_rad"].asDouble(0.0);
    launch_config.longitude_rad = launch["longitude_rad"].asDouble(0.0);
    launch_config.azimuth_rad = launch["azimuth_rad"].asDouble(0.0);
    launch_config.launch_time_s = launch["launch_time_s"].asDouble(0.0);
    launch_config.earth_rotation_angle_rad =
        launch["earth_rotation_angle_rad"].asDouble(0.0);

    const auto& binding_config = config["bindings"];
    if (binding_config.isNull()) {
        throw std::runtime_error("services.soviet_coord.bindings is required.");
    }

    bindings.earth = binding_config["earth"]["name"].asString();
    if (bindings.earth.empty()) {
        throw std::runtime_error("services.soviet_coord.bindings.earth.name is required.");
    }
    bindings.velocity_direction = binding_config["velocity_direction"]["name"].asString();
    bindings.body_attitude = binding_config["body_attitude"]["name"].asString();
    bindings.body_airspeed = binding_config["body_airspeed"]["name"].asString();

    auto service =
        std::make_shared<gnc::services::soviet_coord::SovietCoordService>(launch_config, bindings);
    services.registerService<gnc::services::soviet_coord::ICoordService>(service);

    deferred_actions.push_back(
        [service, registry_scope](gnc::core::ComponentRegistry& component_registry) {
            gnc::core::ScopedRegistry scoped(
                registry_scope, component_registry, "soviet_coord_service");
            service->bindProviders(scoped);
        });
}

} // namespace gnc::bootstrap
