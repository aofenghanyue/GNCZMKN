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

const char* kValidCartesianDynamics = R"json({
  "initial_position": [0.0, 0.0, 1000.0],
  "initial_velocity": [0.0, 0.0, 0.0]
})json";

const char* kValidCartesianInteraction = R"json({
  "acceleration_mps2": [0.0, 0.0, -9.81]
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
            "Stop condition references field 'bad_altitude'",
            "Unknown stop-condition field should be a build error.");

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
