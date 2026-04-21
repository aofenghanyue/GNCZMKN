#include "test_support.hpp"

#include "gnc/core/simulation_builder.hpp"
#include "gnc/plugins/flight_state_3dof/interfaces/i_flight_state_3dof_soviet_observer.hpp"
#include "gnc/plugins/state_3dof/interfaces/i_state_solver_3dof.hpp"

#include <exception>
#include <iostream>

int main() {
    try {
        test_support::registerAvailableComponentTypes();

        const char* mission = R"json(
{
  "simulation": {
    "dt": 0.1,
    "duration": 1.0,
    "integrator": "rk4"
  },
  "form": {
    "components": [
      {
        "type": "state_3dof.point_mass_spherical_soviet",
        "name": "dynamics",
        "config": {
          "launch_azimuth_rad": 1.5707963267948966,
          "initial_state": {
            "longitude_rad": 1.9198621771937625,
            "latitude_rad": 0.5235987755982988,
            "altitude_m": 60000.0,
            "speed_mps": 3200.0,
            "flight_path_angle_rad": -0.1047197551196598,
            "heading_angle_rad": -1.5707963267948966
          }
        }
      },
      {
        "type": "flight_state_3dof.soviet_observer",
        "name": "flight_state",
        "config": {}
      }
    ]
  },
  "environment": {
    "components": [
      { "type": "environment.spherical_earth", "name": "earth", "config": {} },
      { "type": "environment.standard_atmosphere", "name": "atmosphere", "config": {} },
      { "type": "environment.spherical_gravity", "name": "gravity", "config": {} }
    ]
  },
  "vehicle": {
    "common": [
      {
        "type": "cavh.constant_mass",
        "name": "mass",
        "config": {
          "mass_kg": 900.0
        }
      },
      {
        "type": "cavh.aero_table",
        "name": "aero",
        "config": {}
      }
    ],
    "input": [],
    "process": [
      {
        "type": "vehicle.process.programmed_aoa",
        "name": "guidance",
        "config": {
          "bank_angle_deg": 0.0,
          "schedule_altitude_m": [60000, 45000, 30000, 15000],
          "schedule_angle_of_attack_deg": [20, 12, 10, 8]
        }
      }
    ],
    "output": []
  },
  "interaction": {
    "components": [
      {
        "type": "state_3dof_bridge.force_to_local_acceleration_soviet",
        "name": "bridge",
        "config": {}
      }
    ]
  },
  "outputs": {
    "directory": "user/outputs/{timestamp}",
    "format": "csv",
    "session_name": "test_pluginized_build",
    "record": {
      "vehicle.dynamics": "all",
      "vehicle.guidance": "all",
      "vehicle.aero": "all",
      "vehicle.mass": "all",
      "vehicle.flight_state": "all"
    }
  },
  "stop_conditions": [
    {
      "type": "component_field_below",
      "component": "vehicle.dynamics",
      "field": "altitude_m",
      "value": 1000.0,
      "description": "Terminate below 1 km altitude"
    }
  ]
}
)json";

        gnc::core::SimulationBuilder builder;
        test_support::require(builder.loadConfigString(mission),
                              "Pluginized mission JSON could not be parsed.");

        auto& simulator = builder.build();
        auto* dynamics =
            simulator.getRegistry().get<gnc::plugins::state_3dof::IStateSolver3DOF>(
                "vehicle.dynamics");
        test_support::require(dynamics != nullptr,
                              "Dynamics component did not expose IStateSolver3DOF.");
        auto* flight_state = simulator.getRegistry().get<
            gnc::plugins::flight_state_3dof::IFlightState3DOFSovietObserver>(
            "vehicle.flight_state");
        test_support::require(flight_state != nullptr,
                              "Flight-state observer component is missing.");
        const double initial_altitude = dynamics->getAltitude();

        simulator.run();

        test_support::require(simulator.getTerminationReason() == "completed",
                              "Short CAV-H mission should complete without early termination.");
        test_support::require(dynamics->getAltitude() < initial_altitude,
                              "CAV-H mission did not descend during propagation.");
        test_support::require(
            flight_state->getFlightState3DOFSoviet().mach_number > 1.0,
            "Flight-state observer returned an unexpected Mach number after the run.");

        std::cout << "pluginized build checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
