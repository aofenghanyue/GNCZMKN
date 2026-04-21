#include "test_support.hpp"

#include "gnc/core/simulation_builder.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_flight_state_view.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_truth_view.hpp"

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
        "type": "form.local_spherical_3dof.point_mass",
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
        "type": "form.local_spherical_3dof.flight_state_view",
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
        "type": "mass.constant",
        "name": "mass",
        "config": {
          "mass_kg": 900.0
        }
      },
      {
        "type": "aero.table2d",
        "name": "aero",
        "config": {
          "alpha_breaks_rad": [0.17453292519943295, 0.2617993877991494, 0.3490658503988659],
          "mach_breaks": [3.5, 5.0, 8.0, 10.0, 15.0, 20.0, 23.0],
          "lift_coefficients": [
            [0.4500, 0.4250, 0.4000, 0.3800, 0.3700, 0.3600, 0.3500],
            [0.7400, 0.7000, 0.6700, 0.6300, 0.6000, 0.5700, 0.5570],
            [1.0500, 1.0000, 0.9500, 0.9000, 0.8500, 0.8000, 0.7800]
          ],
          "drag_coefficients": [
            [0.2045, 0.1700, 0.1290, 0.1090, 0.1090, 0.1090, 0.1090],
            [0.2960, 0.2630, 0.2240, 0.1970, 0.1950, 0.1920, 0.1920],
            [0.4770, 0.4230, 0.3540, 0.3100, 0.3050, 0.3000, 0.3000]
          ],
          "reference_area_m2": 0.48387,
          "reference_length_m": 3.0
        }
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
        "type": "interaction.local_spherical_3dof.aero_propulsive",
        "name": "interaction",
        "config": {}
      }
    ]
  },
  "outputs": {
    "directory": "user/outputs/{timestamp}",
    "format": "csv",
    "session_name": "test_canonical_build",
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
                              "Canonical mission JSON could not be parsed.");

        auto& simulator = builder.build();
        auto* dynamics =
            simulator.getRegistry().get<gnc::forms::local_spherical_3dof::ITruthView>(
                "vehicle.dynamics");
        test_support::require(dynamics != nullptr,
                              "Dynamics component did not expose local_spherical truth view.");
        auto* flight_state = simulator.getRegistry().get<
            gnc::forms::local_spherical_3dof::IFlightStateView>(
            "vehicle.flight_state");
        test_support::require(flight_state != nullptr,
                              "Flight-state observer component is missing.");
        simulator.initialize();
        const double initial_altitude = dynamics->getLocalSpherical3DoFTruth().state.altitude_m;

        simulator.run();

        test_support::require(simulator.getTerminationReason() == "completed",
                              "Short CAV-H mission should complete without early termination.");
        test_support::require(dynamics->getLocalSpherical3DoFTruth().state.altitude_m <
                                  initial_altitude,
                              "CAV-H mission did not descend during propagation.");
        test_support::require(
            flight_state->getFlightState().mach_number > 1.0,
            "Flight-state observer returned an unexpected Mach number after the run.");

        std::cout << "canonical build checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
