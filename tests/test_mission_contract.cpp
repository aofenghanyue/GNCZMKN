#include "test_support.hpp"

#include "gnc/core/component_factory.hpp"
#include "gnc/core/simulation_builder.hpp"

#include <algorithm>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {

std::vector<std::string> g_stage_trace;

class RecordingComponent : public gnc::core::ComponentBase {
public:
    RecordingComponent(std::string class_name, std::string trace_label)
        : ComponentBase(std::move(class_name)), trace_label_(std::move(trace_label)) {}

    void update(double) override { g_stage_trace.push_back(trace_label_); }

private:
    std::string trace_label_;
};

class EnvironmentProbe final : public RecordingComponent {
public:
    EnvironmentProbe() : RecordingComponent("EnvironmentProbe", "environment") {}
};

class InputProbe final : public RecordingComponent {
public:
    InputProbe() : RecordingComponent("InputProbe", "input") {}
};

class ProcessProbe final : public RecordingComponent {
public:
    ProcessProbe() : RecordingComponent("ProcessProbe", "process") {}
};

class OutputProbe final : public RecordingComponent {
public:
    OutputProbe() : RecordingComponent("OutputProbe", "output") {}
};

class InteractionProbe final : public RecordingComponent {
public:
    InteractionProbe() : RecordingComponent("InteractionProbe", "interaction") {}
};

class InteractionProbeOtherFamily final : public RecordingComponent {
public:
    InteractionProbeOtherFamily()
        : RecordingComponent("InteractionProbeOtherFamily", "interaction_other") {}
};

class FormProbe final : public RecordingComponent {
public:
    FormProbe() : RecordingComponent("FormProbe", "form") {}
};

bool containsSubstring(const std::vector<std::string>& lines, const std::string& needle) {
    return std::any_of(lines.begin(), lines.end(), [&](const std::string& line) {
        return line.find(needle) != std::string::npos;
    });
}

void registerMissionContractTestTypes() {
    using namespace gnc::core;

    auto& factory = ComponentFactory::instance();
    factory.registerType<EnvironmentProbe>("test.environment_probe",
                                           ComponentCategory::Project,
                                           __FILE__,
                                           ComponentPackageRole::Environment,
                                           ExecutionStage::Environment);
    factory.registerType<InputProbe>("test.input_probe",
                                     ComponentCategory::Project,
                                     __FILE__,
                                     ComponentPackageRole::VehicleInput,
                                     ExecutionStage::VehicleInput);
    factory.registerType<ProcessProbe>("test.process_probe",
                                       ComponentCategory::Project,
                                       __FILE__,
                                       ComponentPackageRole::VehicleProcess,
                                       ExecutionStage::VehicleProcess);
    factory.registerType<OutputProbe>("test.output_probe",
                                      ComponentCategory::Project,
                                      __FILE__,
                                      ComponentPackageRole::VehicleOutput,
                                      ExecutionStage::VehicleOutput);
    factory.registerType<InteractionProbe>("test.interaction_probe",
                                           ComponentCategory::Project,
                                           __FILE__,
                                           ComponentPackageRole::Interaction,
                                           ExecutionStage::Interaction,
                                           "test_form");
    factory.registerType<InteractionProbeOtherFamily>("test.interaction_probe_other_family",
                                                      ComponentCategory::Project,
                                                      __FILE__,
                                                      ComponentPackageRole::Interaction,
                                                      ExecutionStage::Interaction,
                                                      "other_form");
    factory.registerType<FormProbe>("test.form_probe",
                                    ComponentCategory::Project,
                                    __FILE__,
                                    ComponentPackageRole::Form,
                                    ExecutionStage::Form,
                                    "test_form");
}

} // namespace

int main() {
    try {
        test_support::registerAvailableComponentTypes();
        registerMissionContractTestTypes();

        const char* legacy_mission = R"json(
{
  "simulation": {
    "dt": 0.1,
    "duration": 0.2
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
                              "Legacy entities[] mission format should now fail fast.");
        test_support::require(
            containsSubstring(legacy_builder.getBuildErrors(),
                              "Legacy top-level 'entities[]' missions"),
            "Legacy mission failure did not explain that entities[] is unsupported.");
        test_support::require(
            containsSubstring(legacy_builder.getBuildErrors(),
                              "form/environment/vehicle/interaction"),
            "Legacy mission failure did not direct the user to the new top-level layout.");

        const char* architecture_mission = R"json(
{
  "simulation": {
    "dt": 0.1,
    "duration": 0.2,
    "integrator": "rk4"
  },
  "form": {
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
  },
  "environment": {},
  "vehicle": {
    "common": [],
    "input": [],
    "process": [],
    "output": []
  },
  "interaction": {
    "components": []
  },
  "outputs": {
    "enabled": false
  }
}
)json";

        gnc::core::SimulationBuilder architecture_builder;
        test_support::require(architecture_builder.loadConfigString(architecture_mission),
                              "Top-level mission JSON could not be parsed.");

        auto& architecture_simulator = architecture_builder.build();
        const auto& component_names = architecture_simulator.getRegistry().getComponentNames();
        test_support::require(
            std::find(component_names.begin(), component_names.end(), "vehicle.dynamics") !=
                component_names.end(),
            "Form components must be registered under the unified vehicle scope.");
        test_support::require(
            std::find(component_names.begin(), component_names.end(), "dynamics") ==
                component_names.end(),
            "Missions should not register bare component names under the new schema.");

        const char* stage_mission = R"json(
{
  "simulation": {
    "dt": 0.1,
    "duration": 0.1
  },
  "form": {
    "components": [
      {
        "type": "test.form_probe",
        "name": "dynamics",
        "config": {}
      }
    ]
  },
  "environment": {
    "components": [
      {
        "type": "test.environment_probe",
        "name": "world",
        "config": {}
      }
    ]
  },
  "vehicle": {
    "common": [],
    "input": [
      {
        "type": "test.input_probe",
        "name": "sensor",
        "config": {}
      }
    ],
    "process": [
      {
        "type": "test.process_probe",
        "name": "guidance",
        "config": {}
      }
    ],
    "output": [
      {
        "type": "test.output_probe",
        "name": "actuator",
        "config": {}
      }
    ]
  },
  "interaction": {
    "components": [
      {
        "type": "test.interaction_probe",
        "name": "bridge",
        "config": {}
      }
    ]
  },
  "outputs": {
    "enabled": false
  }
}
)json";

        gnc::core::SimulationBuilder stage_builder;
        test_support::require(stage_builder.loadConfigString(stage_mission),
                              "Stage-order mission JSON could not be parsed.");

        g_stage_trace.clear();
        auto& stage_simulator = stage_builder.build();
        stage_simulator.run();

        const std::vector<std::string> expected_trace = {
            "environment", "input", "process", "output", "interaction", "form"};
        test_support::require(
            g_stage_trace == expected_trace,
            "Execution stages no longer run in the expected environment -> input -> "
            "process -> output -> interaction -> form order.");

        const char* incompatible_mission = R"json(
{
  "simulation": {
    "dt": 0.1,
    "duration": 0.1
  },
  "form": {
    "components": [
      {
        "type": "test.form_probe",
        "name": "dynamics",
        "config": {}
      }
    ]
  },
  "environment": {},
  "vehicle": {
    "common": [],
    "input": [],
    "process": [],
    "output": []
  },
  "interaction": {
    "components": [
      {
        "type": "test.interaction_probe_other_family",
        "name": "bridge",
        "config": {}
      }
    ]
  },
  "outputs": {
    "enabled": false
  }
}
)json";

        gnc::core::SimulationBuilder incompatible_builder;
        test_support::require(incompatible_builder.loadConfigString(incompatible_mission),
                              "Incompatible mission JSON could not be parsed.");

        bool incompatible_failed = false;
        try {
            incompatible_builder.build();
        } catch (const std::exception&) {
            incompatible_failed = true;
        }

        test_support::require(incompatible_failed,
                              "Missions with incompatible form and interaction families must fail.");
        test_support::require(
            containsSubstring(incompatible_builder.getBuildErrors(), "targets form family"),
            "Incompatible family failure did not explain the form-family mismatch.");

        std::cout << "mission contract checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
