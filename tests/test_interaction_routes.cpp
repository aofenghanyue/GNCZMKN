#include "test_support.hpp"

#include "gnc/core/component_factory.hpp"
#include "gnc/core/simulation_builder.hpp"
#include "gnc/forms/cartesian_3dof/interfaces/i_input_provider.hpp"
#include "gnc/forms/cartesian_3dof/interfaces/i_truth_view.hpp"
#include "gnc/interactions/cartesian_3dof/components/force_accel.hpp"
#include "gnc/vehicle/output/components/constant_force.hpp"
#include "gnc/vehicle/output/interfaces/i_constant_mass.hpp"
#include "gnc/vehicle/output/interfaces/i_continuous_mass.hpp"

#include <exception>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace {

class DualMass final : public gnc::core::ComponentBase,
                       public gnc::vehicle::output::IConstantMass,
                       public gnc::vehicle::output::IContinuousMass {
public:
    DualMass() : ComponentBase("DualMass") {}

    void update(double) override {}
    double getMassKg() const override { return 2.0; }
    double getMassRateKgPerSec() const override { return 0.0; }
};

void registerInteractionRouteTestTypes() {
    using namespace gnc::core;
    ComponentFactory::instance().registerType<DualMass,
                                              gnc::vehicle::output::IConstantMass,
                                              gnc::vehicle::output::IContinuousMass>(
        "test.dual_mass",
        ComponentCategory::Project,
        __FILE__,
        ComponentPackageRole::VehicleOutput,
        ExecutionStage::VehicleOutput);
}

bool containsSubstring(const std::vector<std::string>& lines, const std::string& needle) {
    for (const auto& line : lines) {
        if (line.find(needle) != std::string::npos) {
            return true;
        }
    }
    return false;
}

void requireConfigureFailure(const gnc::core::ConfigNode& config,
                             const std::string& expected_fragment,
                             const std::string& message) {
    bool failed = false;
    try {
        gnc::vehicle::output::ConstantForce force;
        force.configure(config, "force.constant.config");
    } catch (const std::exception& ex) {
        failed = std::string(ex.what()).find(expected_fragment) != std::string::npos;
    }
    test_support::require(failed, message);
}

std::string missionWithOutputAndInteraction(const std::string& output_entries,
                                            const std::string& interaction_config = "{}") {
    return R"json(
{
  "simulation": { "dt": 0.1, "duration": 0.1 },
  "outputs": { "enabled": false },
  "vehicles": [
    {
      "id": "vehicle",
      "form": {
        "components": [
          {
            "type": "form.cartesian_3dof.point_mass",
            "name": "dynamics",
            "config": {
              "initial_position": [0.0, 0.0, 1000.0],
              "initial_velocity": [0.0, 0.0, 0.0]
            }
          }
        ]
      },
      "common": [],
      "input": [],
      "process": [],
      "output": [
)json" + output_entries +
           R"json(
      ],
      "interaction": {
        "components": [
          {
            "type": "interaction.cartesian_3dof.force_accel",
            "name": "interaction",
            "config": )json" + interaction_config +
           R"json(
          }
        ]
      }
    }
  ]
}
)json";
}

bool buildFailsWith(const std::string& mission, const std::string& fragment) {
    gnc::core::SimulationBuilder builder;
    test_support::require(builder.loadConfigString(mission),
                          "Interaction route failure mission JSON did not parse.");
    try {
        builder.build();
    } catch (const std::exception&) {
        return containsSubstring(builder.getBuildErrors(), fragment);
    }
    return false;
}

} // namespace

int main() {
    try {
        test_support::registerBuiltinComponentTypes();
        registerInteractionRouteTestTypes();

        using namespace test_support;
        requireConfigureFailure(object({}),
                                "force_n",
                                "force.constant must require force_n.");
        requireConfigureFailure(object({field("force_n", string("bad"))}),
                                "force_n",
                                "force.constant must reject non-array force_n.");
        requireConfigureFailure(
            object({field("force_n", array({number(1.0), number(2.0)}))}),
            "exactly 3",
            "force.constant must reject force_n arrays with the wrong length.");

        const std::string valid_outputs = R"json(
        {
          "type": "mass.constant",
          "name": "mass",
          "config": {
            "asset_file": "framework/data/vehicles/cavh/output/mass_atmospheric_reference.json"
          }
        },
        {
          "type": "force.constant",
          "name": "force",
          "config": { "force_n": [1800.0, 2700.0, 3600.0] }
        }
)json";

        gnc::core::SimulationBuilder builder;
        test_support::require(
            builder.loadConfigString(missionWithOutputAndInteraction(valid_outputs)),
            "Valid force_accel mission JSON did not parse.");
        auto& simulator = builder.build();
        simulator.initialize();

        auto* truth_view =
            simulator.getRegistry().get<gnc::forms::cartesian_3dof::ITruthView>(
                "vehicle.dynamics");
        auto* input_provider =
            simulator.getRegistry().get<gnc::forms::cartesian_3dof::IInputProvider>(
                "vehicle.interaction");
        test_support::require(truth_view != nullptr && input_provider != nullptr,
                              "force_accel mission did not register required interfaces.");
        const auto input = input_provider->computeCartesian3DoFInput(
            truth_view->getCartesian3DoFTruth(),
            0.0);
        test_support::requireVectorNear(input.acceleration_mps2,
                                        gnc::math::Vector3(2.0, 3.0, 4.0),
                                        1.0e-12,
                                        "force_accel did not compute force / mass.");

        const std::string missing_mass_outputs = R"json(
        {
          "type": "force.constant",
          "name": "force",
          "config": { "force_n": [4.0, 6.0, 8.0] }
        }
)json";
        test_support::require(
            buildFailsWith(missionWithOutputAndInteraction(missing_mass_outputs),
                           "requires exactly one mass provider"),
            "force_accel must fail build when no mass provider is available.");

        const std::string missing_force_outputs = R"json(
        {
          "type": "mass.constant",
          "name": "mass",
          "config": {
            "asset_file": "framework/data/vehicles/cavh/output/mass_atmospheric_reference.json"
          }
        }
)json";
        test_support::require(
            buildFailsWith(missionWithOutputAndInteraction(missing_force_outputs),
                           "IForceProvider"),
            "force_accel must fail build when no force provider is available.");

        const std::string dual_mass_outputs = R"json(
        {
          "type": "test.dual_mass",
          "name": "mass",
          "config": {}
        },
        {
          "type": "force.constant",
          "name": "force",
          "config": { "force_n": [4.0, 6.0, 8.0] }
        }
)json";
        test_support::require(
            buildFailsWith(missionWithOutputAndInteraction(dual_mass_outputs),
                           "requires exactly one mass provider"),
            "force_accel must fail build when one mass component implements both mass interfaces.");

        std::cout << "interaction route checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
