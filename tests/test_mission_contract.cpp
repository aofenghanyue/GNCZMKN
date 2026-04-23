#include "test_support.hpp"

#include "gnc/core/component_factory.hpp"
#include "gnc/core/simulation_builder.hpp"
#include "gnc/forms/cartesian_3dof/interfaces/i_truth_view.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_flight_state_view.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_truth_view.hpp"

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

class CommonProbe final : public RecordingComponent {
public:
    CommonProbe() : RecordingComponent("CommonProbe", "common") {}
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

class CommonStageLeakProbe final : public RecordingComponent {
public:
    CommonStageLeakProbe()
        : RecordingComponent("CommonStageLeakProbe", "common_stage_leak") {}
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
    factory.registerType<CommonProbe>("test.common_probe",
                                      ComponentCategory::Project,
                                      __FILE__,
                                      ComponentPackageRole::VehicleCommon,
                                      ExecutionStage::None);
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
    factory.registerType<CommonStageLeakProbe>("test.common_stage_leak_probe",
                                               ComponentCategory::Project,
                                               __FILE__,
                                               ComponentPackageRole::VehicleCommon,
                                               ExecutionStage::VehicleOutput);
}

} // namespace

int main() {
    try {
        test_support::registerBuiltinComponentTypes();
        test_support::registerStableTestProjectComponentTypes();
        registerMissionContractTestTypes();

        auto& factory = gnc::core::ComponentFactory::instance();
        test_support::require(factory.hasType("test_fixture.process_probe"),
                              "Stable test fixture process probe was not auto-registered.");
        test_support::require(
            factory.getPackageRole("test_fixture.process_probe") ==
                gnc::core::ComponentPackageRole::VehicleProcess,
            "Test fixture process probe did not retain vehicle.process package metadata.");
        test_support::require(
            factory.getExecutionStage("test_fixture.process_probe") ==
                gnc::core::ExecutionStage::VehicleProcess,
            "Test fixture process probe did not retain vehicle.process stage metadata.");
        test_support::require(
            factory.getPackageRole("test_fixture.output_probe") ==
                gnc::core::ComponentPackageRole::VehicleOutput,
            "Test fixture output probe did not retain vehicle.output package metadata.");
        test_support::require(
            factory.getExecutionStage("test_fixture.output_probe") ==
                gnc::core::ExecutionStage::VehicleOutput,
            "Test fixture output probe did not retain vehicle.output stage metadata.");
        test_support::require(
            factory.getFormFamily("test_fixture.local_spherical_output_probe") ==
                "local_spherical_3dof",
            "Test fixture local_spherical output probe did not retain form-family metadata.");

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
          "type": "form.cartesian_3dof.point_mass",
          "name": "dynamics",
          "config": {
            "initial_position": [0.0, 0.0, 1000.0],
            "initial_velocity": [250.0, 0.0, 40.0]
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

        const char* global_service_scope_mission = R"json(
{
  "simulation": {
    "dt": 0.1,
    "duration": 0.1
  },
  "global_services": {
    "coordinate_tree": {
      "spec": "empty"
    }
  },
  "form": {
    "components": [
      {
        "type": "form.cartesian_3dof.point_mass",
        "name": "dynamics",
        "config": {
          "initial_position": [0.0, 0.0, 1000.0],
          "initial_velocity": [250.0, 0.0, 0.0]
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
    "components": [
      {
        "type": "interaction.cartesian_3dof.direct_accel",
        "name": "interaction",
        "config": {
          "acceleration_mps2": [0.0, 0.0, -9.81]
        }
      }
    ]
  },
  "outputs": {
    "enabled": false
  }
}
)json";

        gnc::core::SimulationBuilder global_service_scope_builder;
        test_support::require(
            global_service_scope_builder.loadConfigString(global_service_scope_mission),
            "Global-service-scope mission JSON could not be parsed.");

        bool global_service_scope_failed = false;
        try {
            global_service_scope_builder.build();
        } catch (const std::exception&) {
            global_service_scope_failed = true;
        }

        test_support::require(
            global_service_scope_failed,
            "coordinate_tree under global_services should fail in v1.");
        test_support::require(
            containsSubstring(global_service_scope_builder.getBuildErrors(),
                              "only supported in scope 'vehicle'"),
            "Global coordinate_tree service failure did not report the vehicle-only scope.");

        const char* environment_service_scope_mission = R"json(
{
  "simulation": {
    "dt": 0.1,
    "duration": 0.1
  },
  "form": {
    "components": [
      {
        "type": "form.cartesian_3dof.point_mass",
        "name": "dynamics",
        "config": {
          "initial_position": [0.0, 0.0, 1000.0],
          "initial_velocity": [250.0, 0.0, 0.0]
        }
      }
    ]
  },
  "environment": {
    "services": {
      "coordinate_tree": {
        "spec": "empty"
      }
    }
  },
  "vehicle": {
    "common": [],
    "input": [],
    "process": [],
    "output": []
  },
  "interaction": {
    "components": [
      {
        "type": "interaction.cartesian_3dof.direct_accel",
        "name": "interaction",
        "config": {
          "acceleration_mps2": [0.0, 0.0, -9.81]
        }
      }
    ]
  },
  "outputs": {
    "enabled": false
  }
}
)json";

        gnc::core::SimulationBuilder environment_service_scope_builder;
        test_support::require(
            environment_service_scope_builder.loadConfigString(
                environment_service_scope_mission),
            "Environment-service-scope mission JSON could not be parsed.");

        bool environment_service_scope_failed = false;
        try {
            environment_service_scope_builder.build();
        } catch (const std::exception&) {
            environment_service_scope_failed = true;
        }

        test_support::require(
            environment_service_scope_failed,
            "coordinate_tree under environment.services should fail in v1.");
        test_support::require(
            containsSubstring(environment_service_scope_builder.getBuildErrors(),
                              "only supported in scope 'vehicle'"),
            "Environment coordinate_tree service failure did not report the vehicle-only scope.");

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
        "type": "form.cartesian_3dof.point_mass",
        "name": "dynamics",
        "config": {
          "initial_position": [0.0, 0.0, 1000.0],
          "initial_velocity": [250.0, 0.0, 0.0]
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
    "components": [
      {
        "type": "interaction.cartesian_3dof.direct_accel",
        "name": "interaction",
        "config": {
          "acceleration_mps2": [0.0, 0.0, -9.81]
        }
      }
    ]
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

        const char* project_wrong_block_mission = R"json(
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
        "process": [
      {
        "type": "test_fixture.output_probe",
        "name": "misplaced_output",
        "config": {}
      }
    ],
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

        gnc::core::SimulationBuilder project_wrong_block_builder;
        test_support::require(
            project_wrong_block_builder.loadConfigString(project_wrong_block_mission),
            "Project wrong-block mission JSON could not be parsed.");

        bool project_wrong_block_failed = false;
        try {
            project_wrong_block_builder.build();
        } catch (const std::exception&) {
            project_wrong_block_failed = true;
        }

        test_support::require(
            project_wrong_block_failed,
            "Project components placed in the wrong vehicle block must fail assembly.");
        test_support::require(
            containsSubstring(project_wrong_block_builder.getBuildErrors(),
                              "registered as role 'vehicle_output' but was placed in "
                              "'vehicle.process'"),
            "Project wrong-block failure did not report the registered vehicle role.");

        const char* project_stage_mismatch_mission = R"json(
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
        "process": [
      {
        "type": "test_fixture.process_wrong_stage_probe",
        "name": "bad_stage",
        "config": {}
      }
    ],
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

        gnc::core::SimulationBuilder project_stage_mismatch_builder;
        test_support::require(
            project_stage_mismatch_builder.loadConfigString(project_stage_mismatch_mission),
            "Project stage-mismatch mission JSON could not be parsed.");

        bool project_stage_mismatch_failed = false;
        try {
            project_stage_mismatch_builder.build();
        } catch (const std::exception&) {
            project_stage_mismatch_failed = true;
        }

        test_support::require(
            project_stage_mismatch_failed,
            "Project components with mismatched execution stage metadata must fail assembly.");
        test_support::require(
            containsSubstring(project_stage_mismatch_builder.getBuildErrors(),
                              "registered for stage 'vehicle_output'"),
            "Project stage mismatch did not report the registered execution stage.");
        test_support::require(
            containsSubstring(project_stage_mismatch_builder.getBuildErrors(),
                              "'vehicle.process' which executes at stage 'vehicle_process'"),
            "Project stage mismatch did not report the expected vehicle.process stage.");

        const char* project_form_family_mismatch_mission = R"json(
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
    "output": [
      {
        "type": "test_fixture.local_spherical_output_probe",
        "name": "family_specific_output",
        "config": {}
      }
    ]
  },
  "interaction": {
    "components": []
  },
  "outputs": {
    "enabled": false
  }
}
)json";

        gnc::core::SimulationBuilder project_form_family_mismatch_builder;
        test_support::require(
            project_form_family_mismatch_builder.loadConfigString(
                project_form_family_mismatch_mission),
            "Project form-family mismatch mission JSON could not be parsed.");

        bool project_form_family_mismatch_failed = false;
        try {
            project_form_family_mismatch_builder.build();
        } catch (const std::exception&) {
            project_form_family_mismatch_failed = true;
        }

        test_support::require(
            project_form_family_mismatch_failed,
            "Project components with incompatible form-family metadata must fail assembly.");
        test_support::require(
            containsSubstring(project_form_family_mismatch_builder.getBuildErrors(),
                              "targets form family 'local_spherical_3dof'"),
            "Project form-family mismatch did not report the component family.");
        test_support::require(
            containsSubstring(project_form_family_mismatch_builder.getBuildErrors(),
                              "selected form family is 'test_form'"),
            "Project form-family mismatch did not report the selected mission family.");

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
    "common": [
      {
        "type": "test.common_probe",
        "name": "common_data",
        "config": {}
      }
    ],
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

        const char* invalid_common_stage_mission = R"json(
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
    "common": [
      {
        "type": "test.common_stage_leak_probe",
        "name": "common_data",
        "config": {}
      }
    ],
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

        gnc::core::SimulationBuilder invalid_common_stage_builder;
        test_support::require(
            invalid_common_stage_builder.loadConfigString(invalid_common_stage_mission),
            "Invalid common-stage mission JSON could not be parsed.");

        bool invalid_common_stage_failed = false;
        try {
            invalid_common_stage_builder.build();
        } catch (const std::exception&) {
            invalid_common_stage_failed = true;
        }

        test_support::require(
            invalid_common_stage_failed,
            "vehicle.common should reject components that still advertise a runtime stage.");
        test_support::require(
            containsSubstring(invalid_common_stage_builder.getBuildErrors(),
                              "requires execution stage 'none'"),
            "vehicle.common stage validation did not explain that common is non-scheduled.");

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

        const char* real_family_mismatch_mission = R"json(
{
  "simulation": {
    "dt": 0.1,
    "duration": 0.1
  },
  "form": {
    "components": [
      {
        "type": "form.cartesian_3dof.point_mass",
        "name": "dynamics",
        "config": {
          "initial_position": [0.0, 0.0, 1000.0],
          "initial_velocity": [250.0, 0.0, 0.0]
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
    "components": [
      {
        "type": "interaction.local_spherical_3dof.direct_accel",
        "name": "interaction",
        "config": {
          "local_acceleration_nue_mps2": [0.0, -9.80665, 0.0]
        }
      }
    ]
  },
  "outputs": {
    "enabled": false
  }
}
)json";

        gnc::core::SimulationBuilder real_family_mismatch_builder;
        test_support::require(
            real_family_mismatch_builder.loadConfigString(real_family_mismatch_mission),
            "Real form-family mismatch mission JSON could not be parsed.");

        bool real_family_mismatch_failed = false;
        try {
            real_family_mismatch_builder.build();
        } catch (const std::exception&) {
            real_family_mismatch_failed = true;
        }

        test_support::require(
            real_family_mismatch_failed,
            "Cartesian form should reject a local_spherical_3dof interaction.");
        test_support::require(
            containsSubstring(real_family_mismatch_builder.getBuildErrors(), "cartesian_3dof"),
            "Real form-family mismatch did not mention the selected cartesian_3dof family.");

        const char* direct_accel_mission = R"json(
{
  "simulation": {
    "dt": 0.1,
    "duration": 0.5,
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
            "altitude_m": 10000.0,
            "speed_mps": 500.0,
            "flight_path_angle_rad": -0.05,
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
      { "type": "environment.standard_atmosphere", "name": "atmosphere", "config": {} }
    ]
  },
  "vehicle": {
    "common": [],
    "input": [],
    "process": [],
    "output": []
  },
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
  },
  "outputs": {
    "enabled": false
  }
}
)json";

        gnc::core::SimulationBuilder direct_builder;
        test_support::require(direct_builder.loadConfigString(direct_accel_mission),
                              "Direct-accel local_spherical_3dof mission could not be parsed.");

        auto& direct_simulator = direct_builder.build();
        auto* direct_dynamics =
            direct_simulator.getRegistry().get<gnc::forms::local_spherical_3dof::ITruthView>(
                "vehicle.dynamics");
        auto* direct_flight_state = direct_simulator.getRegistry().get<
            gnc::forms::local_spherical_3dof::IFlightStateView>(
            "vehicle.flight_state");
        test_support::require(direct_dynamics != nullptr,
                              "Direct-accel mission did not expose local_spherical truth view.");
        test_support::require(direct_flight_state != nullptr,
                              "Direct-accel mission did not expose flight-state view.");

        direct_simulator.initialize();
        const double initial_direct_altitude =
            direct_dynamics->getLocalSpherical3DoFTruth().state.altitude_m;
        direct_simulator.run();

        test_support::require(
            direct_dynamics->getLocalSpherical3DoFTruth().state.altitude_m <
                initial_direct_altitude,
                              "Direct-accel local_spherical_3dof mission did not descend.");
        test_support::require(
            direct_flight_state->getFlightState().dynamic_pressure_pa > 0.0,
            "Direct-accel local_spherical_3dof mission returned a non-physical flight state.");

        const char* cartesian_direct_accel_mission = R"json(
{
  "simulation": {
    "dt": 0.1,
    "duration": 0.5,
    "integrator": "rk4"
  },
  "form": {
    "components": [
      {
        "type": "form.cartesian_3dof.point_mass",
        "name": "dynamics",
        "config": {
          "initial_position": [0.0, 0.0, 1000.0],
          "initial_velocity": [250.0, 0.0, 0.0]
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
    "components": [
      {
        "type": "interaction.cartesian_3dof.direct_accel",
        "name": "interaction",
        "config": {
          "acceleration_mps2": [0.0, 0.0, -9.81]
        }
      }
    ]
  },
  "outputs": {
    "enabled": false
  },
  "stop_conditions": [
    {
      "type": "component_field_below",
      "component": "vehicle.dynamics",
      "field": "altitude",
      "value": 999.0,
      "description": "Cartesian direct-accel descent threshold"
    }
  ]
}
)json";

        gnc::core::SimulationBuilder cartesian_builder;
        test_support::require(cartesian_builder.loadConfigString(cartesian_direct_accel_mission),
                              "Cartesian direct-accel mission could not be parsed.");

        auto& cartesian_simulator = cartesian_builder.build();
        auto* cartesian_dynamics =
            cartesian_simulator.getRegistry().get<gnc::forms::cartesian_3dof::ITruthView>(
                "vehicle.dynamics");
        test_support::require(cartesian_dynamics != nullptr,
                              "Cartesian direct-accel mission did not expose cartesian truth view.");

        cartesian_simulator.run();

        test_support::require(
            cartesian_simulator.getTerminationReason() ==
                "Cartesian direct-accel descent threshold",
            "Cartesian direct-accel mission did not trigger the expected descent stop condition.");

        std::cout << "mission contract checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
