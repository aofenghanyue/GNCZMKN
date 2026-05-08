#include "test_support.hpp"

#include "gnc/core/simulation_builder.hpp"
#include "gnc/forms/cartesian_6dof/interfaces/i_input_provider.hpp"

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
      { "type": "environment.standard_atmosphere", "name": "atmosphere", "config": {} }
    ]
  },
  "vehicles": [
    {
      "id": "interceptor",
      "perturbation": {
        "type": "perturbation.static",
        "name": "perturbation",
        "config": { "inputs": {} }
      },
      "form": {
        "components": [
          {
            "type": "form.cartesian_6dof.rigid_body",
            "name": "dynamics",
            "config": {
              "initial_state": {
                "position_m": [0.0, 0.0, 1000.0],
                "velocity_mps": [250.0, 0.0, 0.0],
                "attitude_body_to_frame": [1.0, 0.0, 0.0, 0.0],
                "angular_rate_body_radps": [0.0, 0.0, 0.0]
              }
            }
          }
        ]
      },
      "common": [
        { "type": "vehicle.common.aero_assets_6dof.zero", "name": "aero_assets", "config": {} }
      ],
      "input": [
        { "type": "vehicle.input.cartesian_imu_6dof.ideal", "name": "imu", "config": {} },
        { "type": "vehicle.input.cartesian_satellite_nav_6dof.ideal", "name": "satnav", "config": {} },
        { "type": "vehicle.input.cartesian_air_data_6dof.ideal", "name": "air_data", "config": {} },
        {
          "type": "vehicle.input.cartesian_seeker_6dof.ideal",
          "name": "seeker",
          "config": { "target_truth_lookup_name": "target.truth" }
        }
      ],
      "process": [
        { "type": "vehicle.process.cartesian_phase_sequencer_6dof.ideal", "name": "phase", "config": { "phase_name": "boost" } },
        { "type": "vehicle.process.cartesian_trajectory_planner_6dof.ideal", "name": "planner", "config": {} },
        { "type": "vehicle.process.cartesian_navigation_6dof.ideal", "name": "navigation", "config": {} },
        { "type": "vehicle.process.cartesian_target_tracking_6dof.ideal", "name": "tracking", "config": {} },
        { "type": "vehicle.process.cartesian_guidance_6dof.ideal", "name": "guidance", "config": {} },
        { "type": "vehicle.process.cartesian_attitude_control_6dof.ideal", "name": "attitude_control", "config": {} },
        { "type": "vehicle.process.cartesian_control_allocation_6dof.ideal", "name": "control_allocation", "config": {} }
      ],
      "output": [
        {
          "type": "vehicle.output.mass_properties_6dof.constant",
          "name": "mass",
          "config": {
            "mass_kg": 100.0,
            "center_of_gravity_body_m": [0.0, 0.0, 0.0],
            "inertia_body_kgm2": [10.0, 11.0, 12.0]
          }
        },
        { "type": "vehicle.output.propulsion_6dof.zero", "name": "propulsion", "config": {} },
        { "type": "vehicle.output.actuator_6dof.ideal", "name": "actuator", "priority": 0, "config": {} },
        { "type": "vehicle.output.aerodynamics_6dof.zero", "name": "aero", "priority": 10, "config": {} }
      ],
      "interaction": {
        "components": [
          { "type": "interaction.cartesian_6dof.standard", "name": "interaction", "config": {} }
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
              "initial_position_ecef_m": [3000.0, 0.0, 1000.0],
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
    "session_name": "ideal_cartesian_6dof_mission",
    "record": {
      "interceptor.dynamics": "all",
      "interceptor.imu": "all",
      "interceptor.satnav": "all",
      "interceptor.air_data": "all",
      "interceptor.seeker": "all",
      "interceptor.phase": "all",
      "interceptor.planner": "all",
      "interceptor.navigation": "all",
      "interceptor.tracking": "all",
      "interceptor.guidance": "all",
      "interceptor.attitude_control": "all",
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
    "type": "termination.engagement_cartesian_6dof",
    "name": "termination",
    "config": {
      "min_range_m": 1.0,
      "min_altitude_m": -1.0,
      "max_time_s": 0.1
    }
  },
  "summary": {
    "type": "summary.engagement_cartesian_6dof.ideal",
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
        summary_text.find("Ideal Cartesian 6DOF Engagement Summary") !=
            std::string::npos,
        "Ideal Cartesian 6DOF engagement summary section was missing.");
    test_support::require(summary_text.find("minimum_range_m") != std::string::npos,
                          "Summary did not write minimum_range_m.");
    test_support::require(
        summary_text.find("final_interceptor_speed_mps") != std::string::npos,
        "Summary did not write final interceptor speed.");

    const auto csv_text = readText(output_dir / (session_name + ".csv"));
    const std::vector<std::string> required_columns = {
        "interceptor.dynamics.position.x",
        "interceptor.dynamics.velocity.x",
        "interceptor.dynamics.attitude_body_to_frame.w",
        "interceptor.dynamics.angular_rate_body.x",
        "interceptor.imu.local_acceleration_nue.x",
        "interceptor.imu.angular_rate_body.x",
        "interceptor.satnav.position_ecef.x",
        "interceptor.air_data.mach_number",
        "interceptor.seeker.range_m",
        "interceptor.phase.elapsed_time_s",
        "interceptor.planner.desired_position_ecef.x",
        "interceptor.navigation.mach_number",
        "interceptor.tracking.range_m",
        "interceptor.guidance.line_of_sight_ecef.x",
        "interceptor.attitude_control.body_rate_command.x",
        "interceptor.control_allocation.fin_deflection.x",
        "interceptor.mass.mass_kg",
        "interceptor.propulsion.force_body.x",
        "interceptor.actuator.fin_deflection.x",
        "interceptor.aero.force_body.x",
        "interceptor.aero_assets.sample_count",
        "interceptor.interaction.total_force_body.x",
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
            fs::current_path() / "test_outputs" /
            "ideal_cartesian_6dof_mission";
        std::error_code ec;
        fs::remove_all(output_dir, ec);
        ec.clear();
        fs::create_directories(output_dir, ec);
        test_support::require(!ec,
                              "Failed to prepare output directory: " +
                                  output_dir.generic_string());

        gnc::core::SimulationBuilder builder;
        test_support::require(
            builder.loadConfigString(missionText(output_dir)),
            "Ideal Cartesian 6DOF mission JSON did not parse.");
        auto& simulator = builder.build();

        auto* interaction =
            simulator.getRegistry()
                .get<gnc::forms::cartesian_6dof::IInputProvider>(
                    "interceptor.interaction");
        test_support::require(
            interaction != nullptr,
            "Cartesian 6DOF standard interaction was not registered.");

        simulator.run();
        test_support::require(
            simulator.getTerminationReason().find("time limit") !=
                std::string::npos,
            "Cartesian 6DOF engagement did not terminate with a time-limit reason.");

        checkGeneratedArtifacts(output_dir, "ideal_cartesian_6dof_mission");

        const fs::path example_output_dir =
            fs::current_path() / "user" / "outputs" /
            "example_07_ideal_cartesian_6dof_baseline";
        ec.clear();
        fs::remove_all(example_output_dir, ec);

        const auto example_mission_path =
            sourceRoot() / "user" /
            "example_07_ideal_cartesian_6dof_baseline" / "config" /
            "mission.json";
        gnc::core::SimulationBuilder example_builder;
        test_support::require(
            example_builder.loadConfigString(readText(example_mission_path)),
            "Example ideal Cartesian 6DOF mission JSON did not parse.");
        auto& example_simulator = example_builder.build();
        example_simulator.run();
        test_support::require(
            example_simulator.getTerminationReason().find("time limit") !=
                std::string::npos,
            "Example Cartesian 6DOF mission did not terminate with a time-limit reason.");
        checkGeneratedArtifacts(example_output_dir,
                                "ideal_cartesian_6dof_baseline");

        std::cout << "ideal cartesian 6dof mission checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
