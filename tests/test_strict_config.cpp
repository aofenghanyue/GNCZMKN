#include "test_support.hpp"

#include "gnc/core/component_factory.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/simulation_builder.hpp"
#include "gnc/forms/cartesian_3dof/interfaces/i_input_provider.hpp"

#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {

class NoFamilyInteraction final
    : public gnc::core::ComponentBase,
      public gnc::forms::cartesian_3dof::IInputProvider {
public:
    NoFamilyInteraction() : ComponentBase("NoFamilyInteraction") {}

    void update(double) override {}

    gnc::forms::cartesian_3dof::Input computeCartesian3DoFInput(
        const gnc::forms::cartesian_3dof::Truth&,
        double) const override {
        return {};
    }
};

class RequiredValueComponent final : public gnc::core::ComponentBase {
public:
    RequiredValueComponent() : ComponentBase("RequiredValueComponent") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader(config, config_path).requiredDouble("value");
    }

    void update(double) override {}
};

void registerStrictConfigTestTypes() {
    auto& factory = gnc::core::ComponentFactory::instance();
    factory.registerType<NoFamilyInteraction,
                         gnc::forms::cartesian_3dof::IInputProvider>(
        "test.no_family_interaction",
        gnc::core::ComponentCategory::Project,
        __FILE__,
        gnc::core::ComponentPackageRole::Interaction,
        gnc::core::ExecutionStage::Interaction);
    factory.registerType<RequiredValueComponent>(
        "test.required_input",
        gnc::core::ComponentCategory::Builtin,
        __FILE__,
        gnc::core::ComponentPackageRole::VehicleInput,
        gnc::core::ExecutionStage::VehicleInput);
    factory.registerType<RequiredValueComponent>(
        "test.required_process",
        gnc::core::ComponentCategory::Builtin,
        __FILE__,
        gnc::core::ComponentPackageRole::VehicleProcess,
        gnc::core::ExecutionStage::VehicleProcess);
}

bool containsFragment(const std::vector<std::string>& messages,
                      const std::string& fragment) {
    for (const auto& message : messages) {
        if (message.find(fragment) != std::string::npos) {
            return true;
        }
    }
    return false;
}

void expectBuildFailure(const std::string& json,
                        const std::string& expected_fragment,
                        const std::string& message) {
    gnc::core::SimulationBuilder builder;
    test_support::require(builder.loadConfigString(json),
                          "Strict-config test JSON could not be parsed.");

    bool failed = false;
    std::string exception_text;
    try {
        builder.build();
    } catch (const std::exception& ex) {
        failed = true;
        exception_text = ex.what();
    }

    test_support::require(failed, message + " (build unexpectedly succeeded)");
    test_support::require(
        containsFragment(builder.getBuildErrors(), expected_fragment) ||
            exception_text.find(expected_fragment) != std::string::npos,
        message + " (missing diagnostic fragment: " + expected_fragment + ")");
}

void expectBuildWarning(const std::string& json,
                        const std::string& expected_fragment,
                        const std::string& message) {
    gnc::core::SimulationBuilder builder;
    test_support::require(builder.loadConfigString(json),
                          "Strict-config warning JSON could not be parsed.");

    bool failed = false;
    try {
        builder.build();
    } catch (const std::exception&) {
        failed = true;
    }

    test_support::require(!failed, message + " (build unexpectedly failed)");
    test_support::require(containsFragment(builder.getBuildWarnings(), expected_fragment),
                          message + " (missing warning fragment: " +
                              expected_fragment + ")");
}

void expectBuildSuccess(const std::string& json, const std::string& message) {
    gnc::core::SimulationBuilder builder;
    test_support::require(builder.loadConfigString(json),
                          "Strict-config success JSON could not be parsed.");

    bool failed = false;
    try {
        builder.build();
    } catch (const std::exception&) {
        failed = true;
    }

    test_support::require(!failed, message + " (build unexpectedly failed)");
}

std::string cartesianMissionWith(const std::string& dynamics_config,
                                 const std::string& interaction_type,
                                 const std::string& interaction_config,
                                 const std::string& output_block,
                                 const std::string& stop_block,
                                 const std::string& simulation_block =
                                     R"json("dt": 0.1, "duration": 0.2, "integrator": "rk4")json",
                                 const std::string& outputs_block =
                                     R"json({ "enabled": false })json",
                                 const std::string& input_block = "[]",
                                 const std::string& process_block = "[]") {
    return std::string(R"json(
{
  "simulation": {)json") +
           simulation_block + R"json(},
  "environment": {},
  "vehicles": [
    {
      "id": "vehicle",
      "form": {
        "components": [
          {
            "type": "form.cartesian_3dof.point_mass",
            "name": "dynamics",
            "config": )json" +
           dynamics_config + R"json(
          }
        ]
      },
      "common": [],
      "input": )json" +
           input_block + R"json(,
      "process": )json" +
           process_block + R"json(,
      "output": )json" +
           output_block + R"json(,
      "interaction": {
        "components": [
          {
            "type": ")json" +
           interaction_type + R"json(",
            "name": "interaction",
            "config": )json" +
           interaction_config + R"json(
          }
        ]
      }
    }
  ],
  "outputs": )json" +
           outputs_block +
           stop_block + R"json(
}
)json";
}

std::string cartesianMissionWithEnvironment(const std::string& environment_block,
                                            const std::string& vehicle_services_block =
                                                "{}") {
    return std::string(R"json(
{
  "simulation": { "dt": 0.1, "duration": 0.2, "integrator": "rk4" },
  "environment": )json") +
           environment_block + R"json(,
  "vehicles": [
    {
      "id": "vehicle",
      "services": )json" +
           vehicle_services_block + R"json(,
      "form": {
        "components": [
          {
            "type": "form.cartesian_3dof.point_mass",
            "name": "dynamics",
            "config": )json" +
           R"json({
  "initial_position": [0.0, 0.0, 1000.0],
  "initial_velocity": [0.0, 0.0, 0.0]
})json" + R"json(
          }
        ]
      },
      "common": [],
      "input": [],
      "process": [],
      "output": [],
      "interaction": {
        "components": [
          {
            "type": "interaction.cartesian_3dof.direct_accel",
            "name": "interaction",
            "config": )json" +
           R"json({
  "acceleration_mps2": [0.0, 0.0, -9.81]
})json" + R"json(
          }
        ]
      }
    }
  ],
  "outputs": { "enabled": false }
}
)json";
}

std::string ideal6DofMissionWithConfig(const std::string& imu_config,
                                       const std::string& guidance_config,
                                       const std::string& mass_config,
                                       const std::string& interaction_config,
                                       const std::string& termination_config,
                                       const std::string& summary_config) {
    return std::string(R"json(
{
  "simulation": { "dt": 0.1, "duration": 0.1, "integrator": "rk4" },
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
            "type": "form.local_spherical_6dof.rigid_body",
            "name": "dynamics",
            "config": {
              "initial_state": {
                "longitude_rad": 0.0,
                "latitude_rad": 0.0,
                "altitude_m": 1000.0,
                "velocity_nue_mps": [100.0, 0.0, 0.0],
                "attitude_body_to_nue": [1.0, 0.0, 0.0, 0.0],
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
        { "type": "vehicle.input.imu_6dof.ideal", "name": "imu", "config": )json") +
           imu_config + R"json( },
        { "type": "vehicle.input.satellite_nav_6dof.ideal", "name": "satnav", "config": {} },
        { "type": "vehicle.input.air_data_6dof.ideal", "name": "air_data", "config": {} },
        {
          "type": "vehicle.input.seeker_6dof.ideal",
          "name": "seeker",
          "config": { "target_truth_lookup_name": "target.truth" }
        }
      ],
      "process": [
        { "type": "vehicle.process.phase_sequencer_6dof.ideal", "name": "phase", "config": {} },
        { "type": "vehicle.process.trajectory_planner_6dof.ideal", "name": "planner", "config": {} },
        { "type": "vehicle.process.navigation_6dof.ideal", "name": "navigation", "config": {} },
        { "type": "vehicle.process.target_tracking_6dof.ideal", "name": "tracking", "config": {} },
        { "type": "vehicle.process.guidance_6dof.ideal", "name": "guidance", "config": )json" +
           guidance_config + R"json( },
        { "type": "vehicle.process.attitude_control_6dof.ideal", "name": "attitude_control", "config": {} },
        { "type": "vehicle.process.control_allocation_6dof.ideal", "name": "control_allocation", "config": {} }
      ],
      "output": [
        { "type": "vehicle.output.mass_properties_6dof.constant", "name": "mass", "config": )json" +
           mass_config + R"json( },
        { "type": "vehicle.output.propulsion_6dof.zero", "name": "propulsion", "config": {} },
        { "type": "vehicle.output.actuator_6dof.ideal", "name": "actuator", "config": {} },
        { "type": "vehicle.output.aerodynamics_6dof.zero", "name": "aero", "config": {} }
      ],
      "interaction": {
        "components": [
          {
            "type": "interaction.local_spherical_6dof.standard",
            "name": "interaction",
            "config": )json" +
           interaction_config + R"json(
          }
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
              "initial_position_ecef_m": [6373000.0, 100.0, 0.0],
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
  "outputs": { "enabled": false },
  "termination": {
    "type": "termination.engagement_6dof",
    "name": "termination",
    "config": )json" +
           termination_config + R"json(
  },
  "summary": {
    "type": "summary.engagement_6dof.ideal",
    "name": "summary",
    "config": )json" +
           summary_config + R"json(
  }
}
)json";
}

const char* kValidCartesianDynamics = R"json({
  "initial_position": [0.0, 0.0, 1000.0],
  "initial_velocity": [0.0, 0.0, 0.0]
})json";

const char* kValidCartesianInteraction = R"json({
  "acceleration_mps2": [0.0, 0.0, -9.81]
})json";

const char* kValid6DofMassProperties = R"json({
  "mass_kg": 100.0,
  "center_of_gravity_body_m": [0.0, 0.0, 0.0],
  "inertia_body_kgm2": [10.0, 11.0, 12.0]
})json";

const char* kValid6DofTermination = R"json({
  "min_range_m": 1.0,
  "min_altitude_m": -1.0,
  "max_time_s": 0.1
})json";

} // namespace

int main() {
    try {
        test_support::registerBuiltinComponentTypes();
        registerStrictConfigTestTypes();

        expectBuildFailure(
            cartesianMissionWith(
                R"json({
  "initial_position": [0.0, 0.0, 1000.0],
  "initial_velocity": [0.0, 0.0, 0.0],
  "initial_postion": [1.0, 2.0, 3.0]
})json",
                "interaction.cartesian_3dof.direct_accel",
                kValidCartesianInteraction,
                "[]",
                ""),
            "vehicles[0].form.components[0].config has unrecognized config key",
            "Builtin config typo should be a build error.");

        expectBuildFailure(
            R"json(
{
  "simulation": { "dt": 0.1, "duration": 0.2, "integrator": "rk4" },
  "environment": {},
  "vehicles": [
    {
      "id": "vehicle",
      "form": {
        "components": [
          {
            "type": "form.local_spherical_3dof.point_mass",
            "name": "dynamics",
            "config": {
              "launch_azimuth_rad": 1.5707963267948966,
              "initial_state": {
                "longitude_rad": 1.0,
                "latitude_rad": 0.5,
                "speed_mps": 1000.0,
                "flight_path_angle_rad": 0.0,
                "heading_angle_rad": 0.0
              }
            }
          }
        ]
      },
      "common": [],
      "input": [],
      "process": [],
      "output": [],
      "interaction": {
        "components": [
          {
            "type": "interaction.local_spherical_3dof.direct_accel",
            "name": "interaction",
            "config": {
              "local_acceleration_nue_mps2": [0.0, -9.80665, 0.0]
            }
          }
        ]
      }
    }
  ],
  "outputs": { "enabled": false }
}
)json",
            "vehicles[0].form.components[0].config.initial_state.altitude_m",
            "Missing physical initial-state field should report its path.");

        expectBuildFailure(
            cartesianMissionWith(kValidCartesianDynamics,
                                 "interaction.cartesian_3dof.direct_accel",
                                 kValidCartesianInteraction,
                                 "[]",
                                 "",
                                 R"json("dt": 0.0, "duration": 0.2, "integrator": "rk4")json"),
            "simulation.dt must be > 0",
            "Nonpositive dt should be rejected.");

        expectBuildFailure(
            cartesianMissionWith(kValidCartesianDynamics,
                                 "interaction.cartesian_3dof.direct_accel",
                                 kValidCartesianInteraction,
                                 "[]",
                                 "",
                                 R"json("dt": 0.3, "duration": 1.0, "integrator": "rk4")json"),
            "simulation.duration / simulation.dt must be an integer",
            "Non-integer fixed step count should be rejected.");

        expectBuildFailure(
            cartesianMissionWith(kValidCartesianDynamics,
                                 "interaction.cartesian_3dof.direct_accel",
                                 kValidCartesianInteraction,
                                 "[]",
                                 "",
                                 R"json("dt": 0.1, "duration": 0.2, "integrator": "bogus")json"),
            "Unknown integrator 'bogus'",
            "Unknown integrator should be a build error.");

        expectBuildFailure(
            cartesianMissionWithEnvironment(R"json({
    "components": [
      {
        "type": "environment.spherical_earth",
        "name": "earth",
        "config": { "equatorial_radus_m": 6371000.0 }
      }
    ]
  })json"),
            "environment.components[0].config has unrecognized config key",
            "SphericalEarth typo should be a build error.");

        expectBuildFailure(
            cartesianMissionWithEnvironment(R"json({
    "components": [
      {
        "type": "environment.spherical_earth",
        "name": "earth",
        "config": { "equatorial_radius_m": "6371000" }
      }
    ]
  })json"),
            "environment.components[0].config.equatorial_radius_m must be a number",
            "SphericalEarth wrong value type should be a build error.");

        expectBuildFailure(
            cartesianMissionWithEnvironment(R"json({
    "components": [
      {
        "type": "environment.spherical_gravity",
        "name": "gravity",
        "config": { "sea_level_gravty_mps2": 9.80665 }
      }
    ]
  })json"),
            "environment.components[0].config has unrecognized config key",
            "SphericalGravity typo should be a build error.");

        expectBuildFailure(
            cartesianMissionWithEnvironment(R"json({
    "components": [
      {
        "type": "environment.spherical_gravity",
        "name": "gravity",
        "config": { "sea_level_gravity_mps2": "9.80665" }
      }
    ]
  })json"),
            "environment.components[0].config.sea_level_gravity_mps2 must be a number",
            "SphericalGravity wrong value type should be a build error.");

        expectBuildFailure(
            cartesianMissionWithEnvironment("{}",
                                           R"json({
    "coordinate_tree": {}
  })json"),
            "vehicles[0].services.coordinate_tree.spec",
            "coordinate_tree service should require a string spec.");

        expectBuildFailure(
            cartesianMissionWithEnvironment("{}",
                                           R"json({
    "coordinate_tree": { "spec": 42 }
  })json"),
            "vehicles[0].services.coordinate_tree.spec",
            "coordinate_tree service spec type errors should report the service path.");

        expectBuildFailure(
            cartesianMissionWith(
                kValidCartesianDynamics,
                "interaction.cartesian_3dof.direct_accel",
                kValidCartesianInteraction,
                "[]",
                R"json(,
  "stop_conditions": [
    {
      "type": "component_field_below",
      "component": "vehicle.dynamics",
      "field": "bad_altitude",
      "value": 0.0
    }
  ])json"),
            "Legacy 'stop_conditions[]' are no longer supported",
            "Legacy stop_conditions should be rejected.");

        expectBuildFailure(
            cartesianMissionWith(
                kValidCartesianDynamics,
                "interaction.cartesian_3dof.direct_accel",
                kValidCartesianInteraction,
                "[]",
                R"json(,
  "termination": {
    "type": "termination.component_field_below",
    "name": "termination",
    "config": {
      "component": "vehicle.dynamics",
      "field": "bad_altitude",
      "value": 0.0
    }
  })json"),
            "field 'bad_altitude' not found",
            "Unknown termination field should be a build error.");

        expectBuildFailure(
            cartesianMissionWith(kValidCartesianDynamics,
                                 "interaction.cartesian_3dof.direct_accel",
                                 kValidCartesianInteraction,
                                 R"json([
        {
          "type": "mass.constant",
          "name": "mass",
          "config": {}
        }
      ])json",
                                 ""),
            "vehicles[0].output[0].config.mass_kg",
            "Missing mass_kg should be a build error.");

        expectBuildFailure(
            cartesianMissionWith(kValidCartesianDynamics,
                                 "interaction.cartesian_3dof.direct_accel",
                                 kValidCartesianInteraction,
                                 R"json([
        {
          "type": "mass.constant",
          "name": "mass",
          "config": { "asset_file": 123 }
        }
      ])json",
                                 ""),
            "vehicles[0].output[0].config.asset_file must be a string",
            "asset_file type errors should report the component config path.");

        expectBuildFailure(
            cartesianMissionWith(kValidCartesianDynamics,
                                 "interaction.cartesian_3dof.direct_accel",
                                 kValidCartesianInteraction,
                                 R"json([
        {
          "type": "mass.constant",
          "name": "mass",
          "config": { "asset_file": "tests/assets/phase5/missing_asset.json" }
        }
      ])json",
                                 ""),
            "was not found",
            "Missing asset_file target should be a build error.");

        expectBuildFailure(
            cartesianMissionWith(kValidCartesianDynamics,
                                 "interaction.cartesian_3dof.direct_accel",
                                 kValidCartesianInteraction,
                                 "[]",
                                 "",
                                 R"json("dt": 0.1, "duration": 0.2, "integrator": "rk4")json",
                                 R"json({ "enabled": false })json",
                                 R"json([
        {
          "type": "test.required_input",
          "name": "input_probe",
          "config": {}
        }
      ])json"),
            "vehicles[0].input[0].config.value",
            "Vehicle input config errors should report the array path.");

        expectBuildFailure(
            cartesianMissionWith(kValidCartesianDynamics,
                                 "interaction.cartesian_3dof.direct_accel",
                                 kValidCartesianInteraction,
                                 "[]",
                                 "",
                                 R"json("dt": 0.1, "duration": 0.2, "integrator": "rk4")json",
                                 R"json({ "enabled": false })json",
                                 "[]",
                                 R"json([
        {
          "type": "test.required_process",
          "name": "process_probe",
          "config": {}
        }
      ])json"),
            "vehicles[0].process[0].config.value",
            "Vehicle process config errors should report the array path.");

        expectBuildFailure(
            cartesianMissionWith(kValidCartesianDynamics,
                                 "interaction.cartesian_3dof.direct_accel",
                                 kValidCartesianInteraction,
                                 R"json([
        {
          "type": "aero.table2d",
          "name": "aero",
          "config": {
            "asset_file": "tests/assets/phase5/aero_missing_tables.json"
          }
        }
      ])json",
                                 ""),
            "alpha_breaks_rad",
            "Missing aero table fields should be a build error.");

        expectBuildFailure(
            cartesianMissionWith(kValidCartesianDynamics,
                                 "interaction.cartesian_3dof.direct_accel",
                                 kValidCartesianInteraction,
                                 R"json([
        {
          "type": "aero.table2d",
          "name": "aero",
          "config": {
            "asset_file": "tests/assets/phase5/aero_non_number.json"
          }
        }
      ])json",
                                 ""),
            "row 0 column 1 must be a number",
            "Aero table non-number entries should be a build error.");

        expectBuildFailure(
            cartesianMissionWith(kValidCartesianDynamics,
                                 "test.no_family_interaction",
                                 "{}",
                                 "[]",
                                 ""),
            "must declare a non-empty form family",
            "Interaction components must declare a form family.");

        expectBuildFailure(
            cartesianMissionWith(kValidCartesianDynamics,
                                 "interaction.cartesian_3dof.direct_accel",
                                 kValidCartesianInteraction,
                                 "[]",
                                 "",
                                 R"json("dt": 0.1, "duration": 0.2, "integrator": "rk4")json",
                                 R"json({ "enabled": false })json",
                                 "[]",
                                 R"json([
        {
          "type": "test.required_process",
          "name": "process_probe",
          "priority": 0.5,
          "config": { "value": 1.0 }
        }
      ])json"),
            "vehicles[0].process[0].priority",
            "Fractional priority should be a build error with a precise path.");

        expectBuildSuccess(
            cartesianMissionWith(kValidCartesianDynamics,
                                 "interaction.cartesian_3dof.direct_accel",
                                 kValidCartesianInteraction,
                                 "[]",
                                 "",
                                 R"json("dt": 0.1, "duration": 0.2, "integrator": "rk4")json",
                                 R"json({ "enabled": false })json",
                                 "[]",
                                 R"json([
        {
          "type": "test.required_process",
          "name": "process_probe",
          "rate_hz": 5.0,
          "config": { "value": 1.0 }
        }
      ])json"),
            "Valid rate_hz should build.");

        expectBuildFailure(
            cartesianMissionWith(kValidCartesianDynamics,
                                 "interaction.cartesian_3dof.direct_accel",
                                 kValidCartesianInteraction,
                                 "[]",
                                 "",
                                 R"json("dt": 0.1, "duration": 0.2, "integrator": "rk4")json",
                                 R"json({ "enabled": false })json",
                                 "[]",
                                 R"json([
        {
          "type": "test.required_process",
          "name": "process_probe",
          "rate_hz": 3.0,
          "config": { "value": 1.0 }
        }
      ])json"),
            "rate_hz must divide the simulation frequency",
            "Non-integer rate_hz step interval should be a build error.");

        expectBuildFailure(
            cartesianMissionWith(kValidCartesianDynamics,
                                 "interaction.cartesian_3dof.direct_accel",
                                 kValidCartesianInteraction,
                                 "[]",
                                 "",
                                 R"json("dt": 0.1, "duration": 0.2, "integrator": "rk4")json",
                                 R"json({ "enabled": false })json",
                                 "[]",
                                 R"json([
        {
          "type": "test.required_process",
          "name": "process_probe",
          "rate_hz": 0.0,
          "config": { "value": 1.0 }
        }
      ])json"),
            "rate_hz must be > 0",
            "Nonpositive rate_hz should be a build error.");

        expectBuildFailure(
            cartesianMissionWith(kValidCartesianDynamics,
                                 "interaction.cartesian_3dof.direct_accel",
                                 kValidCartesianInteraction,
                                 "[]",
                                 "",
                                 R"json("dt": 0.1, "duration": 0.2, "integrator": "rk4")json",
                                 R"json({ "enabled": false })json",
                                 "[]",
                                 R"json([
        {
          "type": "test.required_process",
          "name": "process_probe",
          "rate_hz": 20.0,
          "config": { "value": 1.0 }
        }
      ])json"),
            "rate_hz must not exceed",
            "rate_hz above the simulation frequency should be a build error.");

        expectBuildFailure(
            ideal6DofMissionWithConfig(R"json({ "truth_lookup_nam": "dynamics" })json",
                                       "{}",
                                       kValid6DofMassProperties,
                                       "{}",
                                       kValid6DofTermination,
                                       "{}"),
            "vehicles[0].input[0].config has unrecognized config key",
            "6DOF ideal IMU should reject unknown config keys.");

        expectBuildFailure(
            ideal6DofMissionWithConfig("{}",
                                       R"json({ "tracking_lookup_nam": "tracking" })json",
                                       kValid6DofMassProperties,
                                       "{}",
                                       kValid6DofTermination,
                                       "{}"),
            "vehicles[0].process[4].config has unrecognized config key",
            "6DOF guidance should reject unknown config keys.");

        expectBuildFailure(
            ideal6DofMissionWithConfig(
                "{}",
                "{}",
                R"json({
  "mass_kg": 100.0,
  "center_of_gravity_body_m": [0.0, 0.0, 0.0],
  "inertia_body_kgm2": [10.0, 11.0, 12.0],
  "mass_source": "bad"
})json",
                "{}",
                kValid6DofTermination,
                "{}"),
            "vehicles[0].output[0].config has unrecognized config key",
            "6DOF mass properties should reject unknown config keys.");

        expectBuildFailure(
            ideal6DofMissionWithConfig("{}",
                                       "{}",
                                       kValid6DofMassProperties,
                                       R"json({ "force_model": "bad" })json",
                                       kValid6DofTermination,
                                       "{}"),
            "vehicles[0].interaction.components[0].config has unrecognized config key",
            "6DOF standard interaction should reject unknown config keys.");

        expectBuildFailure(
            ideal6DofMissionWithConfig(
                "{}",
                "{}",
                kValid6DofMassProperties,
                "{}",
                R"json({
  "min_range_m": 1.0,
  "min_altitude_m": -1.0,
  "max_time_s": 0.1,
  "stop_reason": "bad"
})json",
                "{}"),
            "termination.config has unrecognized config key",
            "6DOF engagement termination should reject unknown config keys.");

        expectBuildFailure(
            ideal6DofMissionWithConfig("{}",
                                       "{}",
                                       kValid6DofMassProperties,
                                       "{}",
                                       kValid6DofTermination,
                                       R"json({ "summary_mode": "bad" })json"),
            "summary.config has unrecognized config key",
            "6DOF engagement summary should reject unknown config keys.");

        expectBuildWarning(
            cartesianMissionWith(kValidCartesianDynamics,
                                 "interaction.cartesian_3dof.direct_accel",
                                 kValidCartesianInteraction,
                                 "[]",
                                 "",
                                 R"json("dt": 0.1, "duration": 0.2, "integrator": "rk4")json",
                                 R"json({
    "enabled": false,
    "record_initial_satte": true
  })json"),
            "outputs.record_initial_satte",
            "Unknown outputs keys should warn without failing the build.");

        std::cout << "strict config checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
