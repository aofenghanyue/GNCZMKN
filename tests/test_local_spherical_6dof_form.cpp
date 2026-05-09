#include "test_support.hpp"

#include "gnc/core/component_factory.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/simulation_builder.hpp"
#include "gnc/forms/local_spherical_6dof/interfaces/i_input_provider.hpp"
#include "gnc/forms/local_spherical_6dof/interfaces/i_truth_view.hpp"

#include <exception>
#include <iostream>
#include <string>

namespace {

class Constant6DofInput final
    : public gnc::core::ComponentBase,
      public gnc::forms::local_spherical_6dof::IInputProvider {
public:
    Constant6DofInput() : ComponentBase("Constant6DofInput") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        const auto acceleration = reader.requiredDoubleArray("local_acceleration_nue_mps2", 3);
        const auto angular_acceleration =
            reader.requiredDoubleArray("angular_acceleration_body_radps2", 3);
        input_.local_acceleration_nue_mps2 =
            gnc::math::Vector3(acceleration[0], acceleration[1], acceleration[2]);
        input_.angular_acceleration_body_radps2 =
            gnc::math::Vector3(angular_acceleration[0],
                               angular_acceleration[1],
                               angular_acceleration[2]);
        reader.validateNoUnknownKeys();
    }

    void update(double) override {}

    gnc::forms::local_spherical_6dof::Input computeLocalSpherical6DoFInput(
        const gnc::forms::local_spherical_6dof::Truth&,
        double) const override {
        return input_;
    }

private:
    gnc::forms::local_spherical_6dof::Input input_{};
};

void registerTestTypes() {
    auto& factory = gnc::core::ComponentFactory::instance();
    factory.registerType<Constant6DofInput,
                         gnc::forms::local_spherical_6dof::IInputProvider>(
        "test.local_spherical_6dof.acceleration_input",
        gnc::core::ComponentCategory::Project,
        __FILE__,
        gnc::core::ComponentPackageRole::Interaction,
        gnc::core::ExecutionStage::Interaction,
        "local_spherical_6dof");
}

std::string formMission() {
    return R"json(
{
  "simulation": { "dt": 0.1, "duration": 0.1, "integrator": "rk4" },
  "environment": {
    "components": [
      { "type": "environment.spherical_earth", "name": "earth", "config": {} }
    ]
  },
  "vehicles": [
    {
      "id": "vehicle",
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
                "velocity_nue_mps": [300.0, 0.0, 0.0],
                "attitude_body_to_nue": [1.0, 0.0, 0.0, 0.0],
                "angular_rate_body_radps": [0.0, 0.0, 0.0]
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
            "type": "test.local_spherical_6dof.acceleration_input",
            "name": "interaction",
            "config": {
              "local_acceleration_nue_mps2": [0.0, 2.0, 0.0],
              "angular_acceleration_body_radps2": [0.1, 0.0, 0.0]
            }
          }
        ]
      }
    }
  ],
  "outputs": { "enabled": false }
}
)json";
}

} // namespace

int main() {
    try {
        test_support::registerBuiltinComponentTypes();
        registerTestTypes();

        gnc::core::SimulationBuilder builder;
        test_support::require(builder.loadConfigString(formMission()),
                              "6DOF form mission JSON did not parse.");
        auto& simulator = builder.build();
        simulator.initialize();

        auto* truth = simulator.getRegistry()
                          .get<gnc::forms::local_spherical_6dof::ITruthView>(
                              "vehicle.dynamics");
        test_support::require(truth != nullptr, "6DOF truth view not registered.");
        test_support::requireNear(
            truth->getLocalSpherical6DoFTruth().state.altitude_m,
            1000.0,
            1.0e-9,
            "Initial altitude was not published.");

        simulator.run();
        const auto& final_truth = truth->getLocalSpherical6DoFTruth();
        test_support::requireNear(final_truth.state.altitude_m,
                                  1000.0,
                                  1.0e-9,
                                  "Placeholder 6DOF form should not advance altitude.");
        test_support::requireVectorNear(final_truth.state.velocity_nue_mps,
                                        gnc::math::Vector3(300.0, 0.0, 0.0),
                                        1.0e-12,
                                        "Placeholder 6DOF form should not advance velocity.");
        test_support::requireVectorNear(final_truth.state.angular_rate_body_radps,
                                        gnc::math::Vector3::Zero(),
                                        1.0e-12,
                                        "Placeholder 6DOF form should not advance angular rate.");
        test_support::requireVectorNear(final_truth.local_acceleration_nue_mps2,
                                        gnc::math::Vector3(0.0, 2.0, 0.0),
                                        1.0e-12,
                                        "Placeholder 6DOF form did not publish interaction acceleration.");
        test_support::requireVectorNear(final_truth.angular_acceleration_body_radps2,
                                        gnc::math::Vector3(0.1, 0.0, 0.0),
                                        1.0e-12,
                                        "Placeholder 6DOF form did not publish interaction angular acceleration.");

        std::cout << "local_spherical_6dof form checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
