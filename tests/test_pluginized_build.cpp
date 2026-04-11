#include "test_support.hpp"

#include "gnc/core/simulation_builder.hpp"
#include "gnc/plugins/_builtin_plugins.hpp"
#include "gnc/plugins/soviet_coord/interfaces/i_soviet_coord_service.hpp"
#include "gnc/plugins/state_3dof/interfaces/i_state_solver_3dof.hpp"
#include "user/example_02_atmospheric_3dof/components/programmed_aoa_guidance.hpp"

#include <exception>
#include <iostream>

int main() {
    try {
        const char* mission = R"json(
{
  "simulation": {
    "dt": 0.1,
    "duration": 1.0,
    "integrator": "rk4"
  },
  "services": {
    "soviet_coord": {
      "launch": {
        "latitude_rad": 0.5235987755982988,
        "longitude_rad": 1.9198621771937625,
        "azimuth_rad": 1.5707963267948966,
        "launch_time_s": 0.0,
        "earth_rotation_angle_rad": 0.0
      },
      "bindings": {
        "earth": { "name": "earth" },
        "velocity_direction": { "name": "dynamics" }
      }
    }
  },
  "components": [
    { "type": "environment.wgs84_earth", "name": "earth", "config": {} },
    { "type": "environment.standard_atmosphere", "name": "atmosphere", "config": {} },
    { "type": "environment.spherical_gravity", "name": "gravity", "config": {} },
    {
      "type": "example.programmed_aoa",
      "name": "guidance",
      "config": {
        "bank_angle_deg": 0.0,
        "schedule_altitude_m": [60000, 45000, 30000, 15000],
        "schedule_angle_of_attack_deg": [20, 12, 10, 8]
      }
    },
    {
      "type": "aero.simple_polynomial",
      "name": "aero",
      "config": {
        "lift_slope_per_rad": 1.8,
        "drag_zero": 0.09,
        "drag_quadratic": 1.35,
        "reference_area_m2": 0.48,
        "reference_length_m": 2.5
      }
    },
    {
      "type": "state_3dof.point_mass_spherical",
      "name": "dynamics",
      "config": {
        "launch_azimuth_rad": 1.5707963267948966,
        "mass_kg": 900.0,
        "initial_state": {
          "longitude_rad": 1.9198621771937625,
          "latitude_rad": 0.5235987755982988,
          "altitude_m": 60000.0,
          "speed_mps": 3200.0,
          "flight_path_angle_rad": -0.1047197551196598,
          "heading_angle_rad": 1.5707963267948966
        }
      }
    }
  ]
}
)json";

        gnc::core::SimulationBuilder builder;
        test_support::require(builder.loadConfigString(mission),
                              "Pluginized mission JSON could not be parsed.");

        auto& simulator = builder.build();
        auto* coord =
            builder.getGlobalServices().get<gnc::plugins::soviet_coord::ISovietCoordService>();
        test_support::require(coord != nullptr,
                              "Global soviet_coord service was not installed.");
        test_support::require(coord->hasEdge("K", "L"),
                              "Velocity-driven K->L edge is missing in the end-to-end build.");

        auto* dynamics = simulator.getRegistry().get<gnc::plugins::state_3dof::IStateSolver3DOF>(
            "dynamics");
        test_support::require(dynamics != nullptr,
                              "Dynamics component did not expose IStateSolver3DOF.");
        const double initial_altitude = dynamics->getAltitude();

        simulator.run();

        test_support::require(simulator.getTerminationReason() == "completed",
                              "Short atmospheric mission should complete without early termination.");
        test_support::require(dynamics->getAltitude() < initial_altitude,
                              "Atmospheric mission did not descend during propagation.");

        std::cout << "pluginized build checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
