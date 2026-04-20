#include "test_support.hpp"

#include "gnc/core/component_factory.hpp"
#include "gnc/core/plugin_registry.hpp"
#include "gnc/plugins/_builtin_plugins.hpp"

#include <algorithm>
#include <exception>
#include <iostream>
#include <vector>

int main() {
    try {
        using namespace gnc::core;

        auto& factory = ComponentFactory::instance();
        auto& plugins = PluginRegistry::instance();

        const std::vector<std::string> required_component_types = {
            "environment.spherical_earth",
            "environment.wgs84_earth",
            "environment.standard_atmosphere",
            "environment.spherical_gravity",
            "aero.simple_polynomial",
            "mass.continuous_constant_rate",
            "cavh.constant_mass",
            "cavh.aero_table",
            "state_3dof.point_mass_cartesian",
            "state_3dof.point_mass_spherical_soviet",
            "state_3dof_bridge.force_to_local_acceleration_soviet",
            "flight_state_3dof.soviet_observer"};

        for (const auto& type_name : required_component_types) {
            test_support::require(factory.hasType(type_name),
                                  "Missing registered builtin component type: " + type_name);
        }

        const auto registered_plugins = plugins.getPluginNames();
        for (const auto& plugin_name :
             {"environment",
              "aero",
              "mass",
              "cavh",
              "state_3dof",
              "state_3dof_bridge",
              "flight_state_3dof",
              "soviet_coord"}) {
            test_support::require(
                std::find(registered_plugins.begin(), registered_plugins.end(), plugin_name) !=
                    registered_plugins.end(),
                "Missing registered plugin: " + std::string(plugin_name));
        }

        test_support::require(plugins.hasServiceInstaller("soviet_coord"),
                              "soviet_coord service installer was not registered.");
        std::cout << "plugin registry checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
