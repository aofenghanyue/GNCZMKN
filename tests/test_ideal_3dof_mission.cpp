#include "test_support.hpp"

#include "gnc/core/simulation_builder.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_input_provider.hpp"

#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

namespace {

namespace fs = std::filesystem;

std::string readText(const fs::path& path) {
    std::ifstream file(path, std::ios::binary);
    test_support::require(file.is_open(),
                          "Failed to read file: " + path.generic_string());
    std::ostringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

fs::path sourceRoot() {
    return fs::path(__FILE__).parent_path().parent_path();
}

std::string missionText(const fs::path& output_dir) {
    std::ostringstream mission;
    mission << R"json(
{
  "simulation": { "dt": 0.1, "duration": 0.2, "integrator": "rk4" },
  "environment": {
    "components": [
      { "type": "environment.spherical_earth", "name": "earth", "config": {} },
      { "type": "environment.standard_atmosphere", "name": "atmosphere", "config": {} },
      { "type": "environment.spherical_gravity", "name": "gravity", "config": {} }
    ]
  },
  "vehicles": [
    {
      "id": "interceptor",
      "form": {
        "components": [
          {
            "type": "form.local_spherical_3dof.point_mass",
            "name": "dynamics",
            "config": {
              "launch_azimuth_rad": 1.5707963267948966,
              "initial_state": {
                "longitude_rad": 0.0,
                "latitude_rad": 0.0,
                "altitude_m": 1000.0,
                "speed_mps": 250.0,
                "flight_path_angle_rad": 0.0,
                "heading_angle_rad": 1.5707963267948966
              }
            }
          },
          { "type": "form.local_spherical_3dof.flight_state_view", "name": "flight_state", "config": {} }
        ]
      },
      "common": [
        { "type": "vehicle.common.aero_assets_3dof.zero", "name": "aero_assets", "config": {} }
      ],
      "input": [
        { "type": "vehicle.input.imu_3dof.ideal", "name": "imu", "config": {} },
        { "type": "vehicle.input.satellite_nav_3dof.ideal", "name": "satnav", "config": {} },
        { "type": "vehicle.input.air_data_3dof.ideal", "name": "air_data", "config": {} },
        {
          "type": "vehicle.input.seeker_3dof.ideal",
          "name": "seeker",
          "config": { "target_truth_lookup_name": "target.truth" }
        }
      ],
      "process": [
        { "type": "vehicle.process.phase_sequencer_3dof.ideal", "name": "phase", "config": { "phase_name": "midcourse" } },
        { "type": "vehicle.process.trajectory_planner_3dof.ideal", "name": "planner", "config": {} },
        { "type": "vehicle.process.navigation_3dof.ideal", "name": "navigation", "config": {} },
        { "type": "vehicle.process.target_tracking_3dof.ideal", "name": "tracking", "config": {} },
        { "type": "vehicle.process.guidance_3dof.ideal", "name": "guidance", "config": {} },
        { "type": "vehicle.process.flight_control_3dof.ideal", "name": "flight_control", "config": {} },
        { "type": "vehicle.process.control_allocation_3dof.ideal", "name": "control_allocation", "config": {} }
      ],
      "output": [
        { "type": "vehicle.output.mass_3dof.constant", "name": "mass", "config": { "mass_kg": 100.0 } },
        { "type": "vehicle.output.propulsion_3dof.zero", "name": "propulsion", "config": {} },
        { "type": "vehicle.output.actuator_3dof.ideal", "name": "actuator", "priority": 0, "config": {} },
        { "type": "vehicle.output.aerodynamics_3dof.zero", "name": "aero", "priority": 10, "config": {} }
      ],
      "interaction": {
        "components": [
          { "type": "interaction.local_spherical_3dof.standard", "name": "interaction", "config": {} }
        ]
      }
    },
    {
      "id": "target",
      "form": {
        "components": [
          {
            "type": "form.target_point.kinematic",
            "name": "truth",
            "config": {
              "initial_position_ecef_m": [6373000.0, 3000.0, 0.0],
              "velocity_ecef_mps": [0.0, 0.0, 0.0]
            }
          }
        ]
      },
      "common": [],
      "input": [],
      "process": [],
      "output": []
    }
  ],
  "outputs": {
    "directory": ")json"
            << output_dir.generic_string() << R"json(",
    "format": "csv",
    "session_name": "ideal_3dof_mission",
    "record": {
      "interceptor.dynamics": "all",
      "interceptor.flight_state": "all",
      "interceptor.imu": "all",
      "interceptor.satnav": "all",
      "interceptor.air_data": "all",
      "interceptor.seeker": "all",
      "interceptor.phase": "all",
      "interceptor.planner": "all",
      "interceptor.navigation": "all",
      "interceptor.tracking": "all",
      "interceptor.guidance": "all",
      "interceptor.flight_control": "all",
      "interceptor.control_allocation": "all",
      "interceptor.mass": "all",
      "interceptor.propulsion": "all",
      "interceptor.actuator": "all",
      "interceptor.aero": "all",
      "interceptor.aero_assets": "all",
      "interceptor.interaction": "all",
      "target.truth": "all"
    }
  },
  "termination": {
    "type": "termination.engagement_3dof",
    "name": "termination",
    "config": {
      "min_range_m": 1.0,
      "min_altitude_m": -1.0,
      "max_time_s": 0.1
    }
  },
  "summary": {
    "type": "summary.engagement_3dof.ideal",
    "name": "summary",
    "config": {}
  }
}
)json";
    return mission.str();
}

void checkGeneratedArtifacts(const fs::path& output_dir,
                             const std::string& session_name) {
    const auto summary_text = readText(output_dir / "summary.txt");
    test_support::require(
        summary_text.find("Ideal 3DOF Engagement Summary") != std::string::npos,
        "Ideal 3DOF engagement summary section was missing.");
    test_support::require(summary_text.find("minimum_range_m") != std::string::npos,
                          "Summary did not write minimum_range_m.");
    test_support::require(
        summary_text.find("final_interceptor_speed_mps") != std::string::npos,
        "Summary did not write final interceptor speed.");

    const auto csv_text = readText(output_dir / (session_name + ".csv"));
    const std::vector<std::string> required_columns = {
        "interceptor.dynamics.altitude_m",
        "interceptor.flight_state.dynamic_pressure_pa",
        "interceptor.imu.local_acceleration_nue.x",
        "interceptor.satnav.position_ecef.x",
        "interceptor.air_data.mach_number",
        "interceptor.seeker.range_m",
        "interceptor.phase.elapsed_time_s",
        "interceptor.planner.desired_altitude_m",
        "interceptor.navigation.mach_number",
        "interceptor.tracking.range_m",
        "interceptor.guidance.line_of_sight_ecef.x",
        "interceptor.flight_control.angle_of_attack_rad",
        "interceptor.control_allocation.angle_of_attack_rad",
        "interceptor.mass.mass_kg",
        "interceptor.propulsion.force_n.x",
        "interceptor.actuator.angle_of_attack_rad",
        "interceptor.aero.lift_coefficient",
        "interceptor.aero_assets.sample_count",
        "interceptor.interaction.local_acceleration_nue.x",
        "target.truth.position_ecef.x",
    };
    for (const auto& column : required_columns) {
        test_support::require(csv_text.find(column) != std::string::npos,
                              "CSV did not record " + column + ".");
    }
}

} // namespace

int main() {
    try {
        test_support::registerBuiltinComponentTypes();

        const fs::path output_dir =
            fs::current_path() / "test_outputs" / "ideal_3dof_mission";
        std::error_code ec;
        fs::remove_all(output_dir, ec);
        ec.clear();
        fs::create_directories(output_dir, ec);
        test_support::require(!ec,
                              "Failed to prepare output directory: " +
                                  output_dir.generic_string());

        gnc::core::SimulationBuilder builder;
        test_support::require(builder.loadConfigString(missionText(output_dir)),
                              "Ideal 3DOF mission JSON did not parse.");
        auto& simulator = builder.build();

        auto* interaction = simulator.getRegistry()
                                .get<gnc::forms::local_spherical_3dof::IInputProvider>(
                                    "interceptor.interaction");
        test_support::require(interaction != nullptr,
                              "3DOF standard interaction was not registered.");

        simulator.run();
        test_support::require(
            simulator.getTerminationReason().find("time limit") !=
                std::string::npos,
            "3DOF engagement did not terminate with a time-limit reason.");

        checkGeneratedArtifacts(output_dir, "ideal_3dof_mission");

        const fs::path example_output_dir =
            fs::current_path() / "user" / "outputs" /
            "example_05_ideal_3dof_geographic_baseline";
        ec.clear();
        fs::remove_all(example_output_dir, ec);

        const auto example_mission_path =
            sourceRoot() / "user" / "example_05_ideal_3dof_geographic_baseline" /
            "config" / "mission.json";
        gnc::core::SimulationBuilder example_builder;
        test_support::require(
            example_builder.loadConfigString(readText(example_mission_path)),
            "Example ideal 3DOF mission JSON did not parse.");
        auto& example_simulator = example_builder.build();
        example_simulator.run();
        test_support::require(
            example_simulator.getTerminationReason().find("time limit") !=
                std::string::npos,
            "Example 3DOF mission did not terminate with a time-limit reason.");
        checkGeneratedArtifacts(example_output_dir,
                                "ideal_3dof_geographic_baseline");

        std::cout << "ideal 3dof mission checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
