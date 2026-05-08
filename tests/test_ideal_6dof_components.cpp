#include "test_support.hpp"

#include "gnc/core/simulation_builder.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/forms/local_spherical_6dof/interfaces/i_input_provider.hpp"
#include "gnc/forms/target_point/interfaces/i_truth_view.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/input/interfaces/i_air_data_6dof.hpp"
#include "gnc/vehicle/input/interfaces/i_imu_6dof.hpp"
#include "gnc/vehicle/input/interfaces/i_satellite_nav_6dof.hpp"
#include "gnc/vehicle/input/interfaces/i_seeker_6dof.hpp"
#include "gnc/vehicle/common/interfaces/i_aerodynamic_assets_6dof.hpp"
#include "gnc/vehicle/output/interfaces/i_actuator_6dof.hpp"
#include "gnc/vehicle/output/interfaces/i_aerodynamics_6dof.hpp"
#include "gnc/vehicle/output/interfaces/i_mass_properties_6dof.hpp"
#include "gnc/vehicle/output/interfaces/i_propulsion_6dof.hpp"

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

std::string targetPointMission() {
    return R"json(
{
  "simulation": { "dt": 0.1, "duration": 0.2, "integrator": "rk4" },
  "vehicles": [
    {
      "id": "target",
      "form": {
        "components": [
          {
            "type": "form.target_point.kinematic",
            "name": "truth",
            "config": {
              "initial_position_ecef_m": [1000.0, 2000.0, 3000.0],
              "velocity_ecef_mps": [10.0, 0.0, 0.0]
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

std::string outputPrimitiveMission() {
    return R"json(
{
  "simulation": { "dt": 0.1, "duration": 0.1, "integrator": "rk4" },
  "vehicles": [
    {
      "id": "vehicle",
      "form": {
        "components": [
          {
            "type": "form.target_point.kinematic",
            "name": "truth",
            "config": {
              "initial_position_ecef_m": [0.0, 0.0, 0.0],
              "velocity_ecef_mps": [0.0, 0.0, 0.0]
            }
          }
        ]
      },
      "common": [
        { "type": "vehicle.common.aero_assets_6dof.zero", "name": "aero_assets", "config": {} }
      ],
      "input": [],
      "process": [],
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
        { "type": "vehicle.output.actuator_6dof.ideal", "name": "actuator", "config": {} },
        { "type": "vehicle.output.aerodynamics_6dof.zero", "name": "aero", "config": {} }
      ]
    }
  ],
  "outputs": { "enabled": false }
}
)json";
}

std::string sensorMission() {
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
                "angular_rate_body_radps": [0.01, 0.02, 0.03]
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
      "process": [],
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

void checkTargetPointForm() {
    gnc::core::SimulationBuilder builder;
    test_support::require(builder.loadConfigString(targetPointMission()),
                          "Target point mission JSON did not parse.");
    auto& simulator = builder.build();
    simulator.run();

    auto* truth_view =
        simulator.getRegistry().get<gnc::forms::target_point::ITruthView>(
            "target.truth");
    test_support::require(truth_view != nullptr,
                          "Target point truth view was not registered.");
    const auto& truth = truth_view->getTargetPointTruth();
    test_support::requireVectorNear(
        truth.position_ecef_m,
        gnc::math::Vector3(1002.0, 2000.0, 3000.0),
        1.0e-9,
        "Target point position did not advance with constant velocity.");
    test_support::requireVectorNear(
        truth.velocity_ecef_mps,
        gnc::math::Vector3(10.0, 0.0, 0.0),
        1.0e-12,
        "Target point velocity should remain constant.");
}

double observableFieldValue(gnc::interfaces::IObservable* observable,
                            const std::string& field_name) {
    test_support::require(observable != nullptr, "Observable component was missing.");
    for (const auto& field : observable->getObservableFields()) {
        if (field.name == field_name) {
            return field.getter();
        }
    }
    throw std::runtime_error("Observable field not found: " + field_name);
}

void checkOutputPrimitives() {
    gnc::core::SimulationBuilder builder;
    test_support::require(builder.loadConfigString(outputPrimitiveMission()),
                          "6DOF output primitive mission JSON did not parse.");
    auto& simulator = builder.build();
    simulator.run();

    auto* mass = simulator.getRegistry()
                     .get<gnc::vehicle::output::IMassProperties6Dof>("vehicle.mass");
    test_support::require(mass != nullptr, "6DOF mass interface was missing.");
    const auto mass_properties = mass->massProperties6Dof();
    test_support::requireNear(mass_properties.mass_kg,
                              100.0,
                              1.0e-12,
                              "Constant mass 6DOF component returned wrong mass.");
    test_support::requireNear(mass_properties.inertia_body_kgm2(1, 1),
                              11.0,
                              1.0e-12,
                              "Constant mass 6DOF component returned wrong inertia.");

    auto* propulsion = simulator.getRegistry()
                           .get<gnc::vehicle::output::IPropulsion6Dof>(
                               "vehicle.propulsion");
    test_support::require(propulsion != nullptr,
                          "6DOF propulsion interface was missing.");
    test_support::requireVectorNear(
        propulsion->propulsionForceMoment6Dof().force_body_n,
        gnc::math::Vector3::Zero(),
        1.0e-12,
        "Zero propulsion should return zero force.");

    auto* actuator = simulator.getRegistry()
                         .get<gnc::vehicle::output::IActuator6Dof>(
                             "vehicle.actuator");
    test_support::require(actuator != nullptr,
                          "6DOF actuator interface was missing.");
    test_support::requireNear(actuator->actuatorState6Dof().fin_deflection_rad.x(),
                              0.0,
                              1.0e-12,
                              "Ideal actuator default fin command should be zero.");

    auto* aero = simulator.getRegistry()
                     .get<gnc::vehicle::output::IAerodynamics6Dof>("vehicle.aero");
    test_support::require(aero != nullptr, "6DOF aero interface was missing.");
    test_support::requireVectorNear(
        aero->aerodynamicForceMoment6Dof().moment_body_nm,
        gnc::math::Vector3::Zero(),
        1.0e-12,
        "Zero aerodynamics should return zero moment.");

    auto* aero_assets = simulator.getRegistry()
                            .get<gnc::vehicle::common::IAerodynamicAssets6Dof>(
                                "vehicle.aero_assets");
    test_support::require(aero_assets != nullptr,
                          "6DOF aero asset interface was missing.");
    (void)aero_assets;

    auto* aero_asset_observable =
        simulator.getRegistry().get<gnc::interfaces::IObservable>(
            "vehicle.aero_assets");
    test_support::requireNear(observableFieldValue(aero_asset_observable,
                                                   "sample_count"),
                              2.0,
                              1.0e-12,
                              "Zero aero asset should be sampled once per output update.");
}

void checkIdealSensors() {
    gnc::core::SimulationBuilder builder;
    test_support::require(builder.loadConfigString(sensorMission()),
                          "6DOF sensor mission JSON did not parse.");
    auto& simulator = builder.build();
    simulator.run();

    auto* imu =
        simulator.getRegistry().get<gnc::vehicle::input::IImu6Dof>(
            "interceptor.imu");
    test_support::require(imu != nullptr, "Ideal IMU interface was missing.");
    test_support::requireVectorNear(imu->imuMeasurement6Dof().angular_rate_body_radps,
                                    gnc::math::Vector3(0.01, 0.02, 0.03),
                                    1.0e-12,
                                    "Ideal IMU did not report truth angular rate.");

    auto* satnav =
        simulator.getRegistry().get<gnc::vehicle::input::ISatelliteNav6Dof>(
            "interceptor.satnav");
    test_support::require(satnav != nullptr,
                          "Ideal satellite navigation interface was missing.");
    test_support::requireNear(satnav->satelliteNavMeasurement6Dof().altitude_m,
                              1000.0,
                              1.0e-9,
                              "Ideal satellite navigation did not report truth altitude.");

    auto* air_data =
        simulator.getRegistry().get<gnc::vehicle::input::IAirData6Dof>(
            "interceptor.air_data");
    test_support::require(air_data != nullptr,
                          "Ideal air data interface was missing.");
    test_support::require(air_data->airDataMeasurement6Dof().density_kg_per_m3 > 0.0,
                          "Ideal air data did not sample atmosphere density.");

    auto* seeker =
        simulator.getRegistry().get<gnc::vehicle::input::ISeeker6Dof>(
            "interceptor.seeker");
    test_support::require(seeker != nullptr, "Ideal seeker interface was missing.");
    test_support::require(seeker->seekerMeasurement6Dof().range_m > 0.0,
                          "Ideal seeker did not compute target range.");
}

} // namespace

int main() {
    try {
        test_support::registerBuiltinComponentTypes();
        registerTestTypes();
        checkTargetPointForm();
        checkOutputPrimitives();
        checkIdealSensors();
        std::cout << "ideal 6dof component checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
