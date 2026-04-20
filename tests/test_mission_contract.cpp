#include "test_support.hpp"

#include "gnc/core/simulation_builder.hpp"
#include "gnc/plugins/_builtin_plugins.hpp"

#include <algorithm>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {

bool containsSubstring(const std::vector<std::string>& lines, const std::string& needle) {
    return std::any_of(lines.begin(), lines.end(), [&](const std::string& line) {
        return line.find(needle) != std::string::npos;
    });
}

} // namespace

int main() {
    try {
        const char* legacy_mission = R"json(
{
  "simulation": {
    "dt": 0.1,
    "duration": 0.2
  },
  "outputs": {
    "enabled": false
  },
  "components": [
    {
      "type": "state_3dof.point_mass_cartesian",
      "name": "dynamics",
      "config": {
        "initial_position": [0.0, 0.0, 1000.0],
        "initial_velocity": [250.0, 0.0, 40.0],
        "constant_acceleration": [0.0, 0.0, -9.81]
      }
    }
  ]
}
)json";

        gnc::core::SimulationBuilder legacy_builder;
        test_support::require(legacy_builder.loadConfigString(legacy_mission),
                              "Legacy mission JSON could not be parsed.");

        bool legacy_failed = false;
        try {
            legacy_builder.build();
        } catch (const std::exception&) {
            legacy_failed = true;
        }

        test_support::require(legacy_failed,
                              "Legacy root-level mission format should now fail fast.");
        test_support::require(
            containsSubstring(legacy_builder.getBuildErrors(),
                              "Legacy root-level mission format"),
            "Legacy mission failure did not explain that root-level mission format is unsupported.");
        test_support::require(
            containsSubstring(legacy_builder.getBuildErrors(), "entities[]"),
            "Legacy mission failure did not direct the user to migrate to entities[].");

        const char* entity_mission = R"json(
{
  "simulation": {
    "dt": 0.1,
    "duration": 0.2,
    "integrator": "rk4"
  },
  "outputs": {
    "enabled": false
  },
  "entities": [
    {
      "id": "vehicle",
      "role": "vehicle",
      "components": [
        {
          "type": "state_3dof.point_mass_cartesian",
          "name": "dynamics",
          "config": {
            "initial_position": [0.0, 0.0, 1000.0],
            "initial_velocity": [250.0, 0.0, 40.0],
            "constant_acceleration": [0.0, 0.0, -9.81]
          }
        }
      ]
    }
  ]
}
)json";

        gnc::core::SimulationBuilder entity_builder;
        test_support::require(entity_builder.loadConfigString(entity_mission),
                              "Entity-first mission JSON could not be parsed.");

        auto& simulator = entity_builder.build();
        const auto& component_names = simulator.getRegistry().getComponentNames();
        test_support::require(
            std::find(component_names.begin(), component_names.end(), "vehicle.dynamics") !=
                component_names.end(),
            "Single-vehicle missions must keep the unified <entity_id>.<name> component naming.");
        test_support::require(
            std::find(component_names.begin(), component_names.end(), "dynamics") ==
                component_names.end(),
            "Single-vehicle missions should not register a legacy bare component name.");

        std::cout << "mission contract checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
