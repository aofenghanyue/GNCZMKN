#include "test_support.hpp"

#include "gnc/core/component_factory.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/simulation_builder.hpp"
#include "gnc/forms/local_spherical_6dof/interfaces/i_input_provider.hpp"
#include "gnc/vehicle/process/interfaces/i_attitude_control_6dof.hpp"
#include "gnc/vehicle/process/interfaces/i_control_allocation_6dof.hpp"
#include "gnc/vehicle/process/interfaces/i_guidance_6dof.hpp"
#include "gnc/vehicle/process/interfaces/i_navigation_6dof.hpp"
#include "gnc/vehicle/process/interfaces/i_phase_sequencer_6dof.hpp"
#include "gnc/vehicle/process/interfaces/i_target_tracking_6dof.hpp"
#include "gnc/vehicle/process/interfaces/i_trajectory_planner_6dof.hpp"

#include <exception>
#include <iostream>
#include <string>

namespace {

class Zero6DofInput final
    : public gnc::core::ComponentBase,
      public gnc::forms::local_spherical_6dof::IInputProvider {
public:
    Zero6DofInput() : ComponentBase("Zero6DofInput") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader(config, config_path).validateNoUnknownKeys();
    }

    void update(double) override {}

    gnc::forms::local_spherical_6dof::Input computeLocalSpherical6DoFInput(
        const gnc::forms::local_spherical_6dof::Truth&,
        double) const override {
        return {};
    }
};

void registerTestTypes() {
    auto& factory = gnc::core::ComponentFactory::instance();
    factory.registerType<Zero6DofInput,
                         gnc::forms::local_spherical_6dof::IInputProvider>(
        "test.local_spherical_6dof.zero_input",
        gnc::core::ComponentCategory::Project,
        __FILE__,
        gnc::core::ComponentPackageRole::Interaction,
        gnc::core::ExecutionStage::Interaction,
        "local_spherical_6dof");
}

std::string processMission() {
    return R"json(
{
  "simulation": { "dt": 0.1, "duration": 0.1, "integrator": "rk4" },
  "environment": {
    "components": [
      { "type": "environment.spherical_earth", "name": "earth", "config": {} },
      { "type": "environment.standard_atmosphere", "name": "atmosphere", "config": {} }
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
      "common": [],
      "input": [
        { "type": "vehicle.input.imu_6dof.ideal", "name": "imu", "config": {} },
        { "type": "vehicle.input.satellite_nav_6dof.ideal", "name": "satnav", "config": {} },
        { "type": "vehicle.input.air_data_6dof.ideal", "name": "air_data", "config": {} },
        {
          "type": "vehicle.input.seeker_6dof.ideal",
          "name": "seeker",
          "config": { "target_truth_lookup_name": "target.truth" }
        }
      ],
      "process": [
        {
          "type": "vehicle.process.phase_sequencer_6dof.ideal",
          "name": "phase",
          "config": { "phase_name": "boost" }
        },
        { "type": "vehicle.process.trajectory_planner_6dof.ideal", "name": "planner", "config": {} },
        { "type": "vehicle.process.navigation_6dof.ideal", "name": "navigation", "config": {} },
        { "type": "vehicle.process.target_tracking_6dof.ideal", "name": "tracking", "config": {} },
        { "type": "vehicle.process.guidance_6dof.ideal", "name": "guidance", "config": {} },
        { "type": "vehicle.process.attitude_control_6dof.ideal", "name": "attitude_control", "config": {} },
        { "type": "vehicle.process.control_allocation_6dof.ideal", "name": "control_allocation", "config": {} }
      ],
      "output": [],
      "interaction": {
        "components": [
          {
            "type": "test.local_spherical_6dof.zero_input",
            "name": "interaction",
            "config": {}
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
  "outputs": { "enabled": false }
}
)json";
}

void checkIdealProcessChain() {
    gnc::core::SimulationBuilder builder;
    test_support::require(builder.loadConfigString(processMission()),
                          "6DOF process mission JSON did not parse.");
    auto& simulator = builder.build();
    simulator.run();

    auto* phase =
        simulator.getRegistry().get<gnc::vehicle::process::IPhaseSequencer6Dof>(
            "interceptor.phase");
    test_support::require(phase != nullptr, "Phase sequencer interface was missing.");
    test_support::require(phase->phaseState6Dof().phase_name == "boost",
                          "Ideal phase sequencer did not publish configured phase.");

    auto* planner =
        simulator.getRegistry().get<gnc::vehicle::process::ITrajectoryPlanner6Dof>(
            "interceptor.planner");
    test_support::require(planner != nullptr,
                          "Trajectory planner interface was missing.");
    test_support::require(planner->trajectoryPlan6Dof()
                              .desired_position_ecef_m.norm() > 0.0,
                          "Ideal trajectory planner did not copy current position.");

    auto* navigation =
        simulator.getRegistry().get<gnc::vehicle::process::INavigation6Dof>(
            "interceptor.navigation");
    test_support::require(navigation != nullptr,
                          "Navigation interface was missing.");
    test_support::requireNear(navigation->navigationSolution6Dof().altitude_m,
                              1000.0,
                              1.0e-9,
                              "Ideal navigation did not copy satellite altitude.");

    auto* tracking =
        simulator.getRegistry().get<gnc::vehicle::process::ITargetTracking6Dof>(
            "interceptor.tracking");
    test_support::require(tracking != nullptr,
                          "Target tracking interface was missing.");
    test_support::require(tracking->targetTrack6Dof().range_m > 0.0,
                          "Ideal target tracking did not copy seeker range.");

    auto* guidance =
        simulator.getRegistry().get<gnc::vehicle::process::IGuidance6Dof>(
            "interceptor.guidance");
    test_support::require(guidance != nullptr, "Guidance interface was missing.");
    test_support::require(guidance->guidanceCommand6Dof().line_of_sight_ecef.norm() >
                              0.0,
                          "Ideal guidance did not publish line of sight.");

    auto* attitude_control =
        simulator.getRegistry().get<gnc::vehicle::process::IAttitudeControl6Dof>(
            "interceptor.attitude_control");
    test_support::require(attitude_control != nullptr,
                          "Attitude control interface was missing.");
    test_support::requireVectorNear(
        attitude_control->attitudeControlCommand6Dof().moment_command_body_nm,
        gnc::math::Vector3::Zero(),
        1.0e-12,
        "Ideal attitude control should publish zero moment command.");

    auto* allocation =
        simulator.getRegistry().get<gnc::vehicle::process::IControlAllocation6Dof>(
            "interceptor.control_allocation");
    test_support::require(allocation != nullptr,
                          "Control allocation interface was missing.");
    test_support::requireVectorNear(
        allocation->controlAllocationCommand6Dof().control_surfaces.fin_deflection_rad,
        gnc::math::Vector3::Zero(),
        1.0e-12,
        "Ideal control allocation should publish zero fin deflection.");
}

} // namespace

int main() {
    try {
        test_support::registerBuiltinComponentTypes();
        registerTestTypes();
        checkIdealProcessChain();
        std::cout << "ideal 6dof process checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
