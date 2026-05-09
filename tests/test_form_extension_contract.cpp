#include "test_support.hpp"

#include "gnc/core/component_factory.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/core/simulation_builder.hpp"
#include "gnc/core/state_layout.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_continuous_system.hpp"
#include "gnc/interfaces/i_observable.hpp"

#include <algorithm>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {

namespace contract_6dof {

struct Input {
    gnc::math::Vector3 force_n = gnc::math::Vector3::Zero();
    gnc::math::Vector3 moment_nm = gnc::math::Vector3::Zero();
};

struct Truth {
    gnc::math::Vector3 position_m = gnc::math::Vector3::Zero();
    gnc::math::Vector3 velocity_mps = gnc::math::Vector3::Zero();
    gnc::math::Vector3 force_n = gnc::math::Vector3::Zero();
    gnc::math::Vector3 moment_nm = gnc::math::Vector3::Zero();
    double sample_time_s = 0.0;
};

class IInputProvider {
public:
    virtual ~IInputProvider() = default;
    virtual Input computeContract6DoFInput(const Truth& truth,
                                           double time_s) const = 0;
};

class ITruthView {
public:
    virtual ~ITruthView() = default;
    virtual const Truth& getContract6DoFTruth() const = 0;
};

struct ForceMomentCommand {
    gnc::math::Vector3 force_n = gnc::math::Vector3::Zero();
    gnc::math::Vector3 moment_nm = gnc::math::Vector3::Zero();
};

class IForceMomentCommandProvider {
public:
    virtual ~IForceMomentCommandProvider() = default;
    virtual ForceMomentCommand forceMomentCommand() const = 0;
};

class Form final : public gnc::core::ComponentBase,
                   public gnc::interfaces::IContinuousSystem,
                   public ITruthView,
                   public gnc::interfaces::IObservable {
public:
    Form() : ComponentBase("Contract6DoFForm") {
        position_x_index_ = layout_.addVariable("position_x");
        position_y_index_ = layout_.addVariable("position_y");
        position_z_index_ = layout_.addVariable("position_z");
        velocity_x_index_ = layout_.addVariable("velocity_x");
        velocity_y_index_ = layout_.addVariable("velocity_y");
        velocity_z_index_ = layout_.addVariable("velocity_z");
        roll_rate_index_ = layout_.addVariable("roll_rate");
        pitch_rate_index_ = layout_.addVariable("pitch_rate");
        yaw_rate_index_ = layout_.addVariable("yaw_rate");
        state_ = Eigen::VectorXd::Zero(layout_.dimension());
        initial_state_ = state_;
    }

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader(config, config_path).validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        input_provider_ = registry.requireByName<IInputProvider>("interaction");
    }

    void initialize() override { publish(getSimTime()); }
    void publish(double time) override { truth_ = sampleTruth(state_, time); }
    void update(double) override {}

    const gnc::core::StateLayout& getStateLayout() const override { return layout_; }

    void computeDerivatives(double time,
                            const Eigen::VectorXd& state,
                            Eigen::VectorXd& derivative) const override {
        const Truth truth = sampleTruth(state, time);
        const Input input = input_provider_
                                ? input_provider_->computeContract6DoFInput(truth, time)
                                : Input{};
        derivative = Eigen::VectorXd::Zero(layout_.dimension());
        derivative[position_x_index_] = state[velocity_x_index_];
        derivative[position_y_index_] = state[velocity_y_index_];
        derivative[position_z_index_] = state[velocity_z_index_];
        derivative[velocity_x_index_] = input.force_n.x();
        derivative[velocity_y_index_] = input.force_n.y();
        derivative[velocity_z_index_] = input.force_n.z();
        derivative[roll_rate_index_] = input.moment_nm.x();
        derivative[pitch_rate_index_] = input.moment_nm.y();
        derivative[yaw_rate_index_] = input.moment_nm.z();
    }

    const Eigen::VectorXd& getState() const override { return state_; }
    void setState(const Eigen::VectorXd& state) override { state_ = state; }
    Eigen::VectorXd getInitialState() const override { return initial_state_; }

    const Truth& getContract6DoFTruth() const override { return truth_; }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addVector3("position", [this]() -> const gnc::math::Vector3& {
            return truth_.position_m;
        });
        builder.addVector3("velocity", [this]() -> const gnc::math::Vector3& {
            return truth_.velocity_mps;
        });
        builder.addVector3("force", [this]() -> const gnc::math::Vector3& {
            return truth_.force_n;
        });
        builder.addVector3("moment", [this]() -> const gnc::math::Vector3& {
            return truth_.moment_nm;
        });
        return builder.build();
    }

private:
    Truth sampleTruth(const Eigen::VectorXd& state, double time) const {
        Truth truth;
        truth.position_m = gnc::math::Vector3(
            state[position_x_index_],
            state[position_y_index_],
            state[position_z_index_]);
        truth.velocity_mps = gnc::math::Vector3(
            state[velocity_x_index_],
            state[velocity_y_index_],
            state[velocity_z_index_]);
        if (input_provider_) {
            const auto input = input_provider_->computeContract6DoFInput(truth, time);
            truth.force_n = input.force_n;
            truth.moment_nm = input.moment_nm;
        }
        truth.sample_time_s = time;
        return truth;
    }

    gnc::core::StateLayout layout_;
    Eigen::VectorXd state_;
    Eigen::VectorXd initial_state_;
    IInputProvider* input_provider_ = nullptr;
    Truth truth_{};
    int position_x_index_ = -1;
    int position_y_index_ = -1;
    int position_z_index_ = -1;
    int velocity_x_index_ = -1;
    int velocity_y_index_ = -1;
    int velocity_z_index_ = -1;
    int roll_rate_index_ = -1;
    int pitch_rate_index_ = -1;
    int yaw_rate_index_ = -1;
};

class ForceMomentCommandComponent final
    : public gnc::core::ComponentBase,
      public IForceMomentCommandProvider {
public:
    ForceMomentCommandComponent() : ComponentBase("ForceMomentCommandComponent") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        const auto force = reader.requiredDoubleArray("force_n", 3);
        const auto moment = reader.requiredDoubleArray("moment_nm", 3);
        command_.force_n = gnc::math::Vector3(force[0], force[1], force[2]);
        command_.moment_nm = gnc::math::Vector3(moment[0], moment[1], moment[2]);
        reader.validateNoUnknownKeys();
    }

    void update(double) override {}
    ForceMomentCommand forceMomentCommand() const override { return command_; }

private:
    ForceMomentCommand command_{};
};

class ForceMomentInteraction final : public gnc::core::ComponentBase,
                                     public IInputProvider {
public:
    ForceMomentInteraction() : ComponentBase("ForceMomentInteraction") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader(config, config_path).validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        command_provider_ =
            registry.requireByName<IForceMomentCommandProvider>(
                "force_moment_command");
    }

    void update(double) override {}

    Input computeContract6DoFInput(const Truth&, double) const override {
        const auto command = command_provider_->forceMomentCommand();
        return Input{command.force_n, command.moment_nm};
    }

private:
    IForceMomentCommandProvider* command_provider_ = nullptr;
};

} // namespace contract_6dof

class FormWithoutFamily final : public gnc::core::ComponentBase {
public:
    FormWithoutFamily() : ComponentBase("FormWithoutFamily") {}
    void update(double) override {}
};

class OtherFamilyForm final : public gnc::core::ComponentBase {
public:
    OtherFamilyForm() : ComponentBase("OtherFamilyForm") {}
    void update(double) override {}
};

bool containsSubstring(const std::vector<std::string>& lines, const std::string& needle) {
    return std::any_of(lines.begin(), lines.end(), [&](const std::string& line) {
        return line.find(needle) != std::string::npos;
    });
}

void registerContractFormTestTypes() {
    using namespace gnc::core;
    auto& factory = ComponentFactory::instance();
    factory.registerType<contract_6dof::Form,
                         gnc::interfaces::IContinuousSystem,
                         contract_6dof::ITruthView,
                         gnc::interfaces::IObservable>(
        "test.contract_6dof.form",
        ComponentCategory::Project,
        __FILE__,
        ComponentPackageRole::Form,
        ExecutionStage::Form,
        "contract_6dof");
    factory.registerType<contract_6dof::ForceMomentCommandComponent,
                         contract_6dof::IForceMomentCommandProvider>(
        "test.contract_6dof.force_moment_command",
        ComponentCategory::Project,
        __FILE__,
        ComponentPackageRole::VehicleProcess,
        ExecutionStage::VehicleProcess);
    factory.registerType<contract_6dof::ForceMomentInteraction,
                         contract_6dof::IInputProvider>(
        "test.contract_6dof.force_moment_interaction",
        ComponentCategory::Project,
        __FILE__,
        ComponentPackageRole::Interaction,
        ExecutionStage::Interaction,
        "contract_6dof");
    factory.registerType<FormWithoutFamily>(
        "test.form_without_family",
        ComponentCategory::Project,
        __FILE__,
        ComponentPackageRole::Form,
        ExecutionStage::Form,
        "");
    factory.registerType<OtherFamilyForm>(
        "test.other_family_form",
        ComponentCategory::Project,
        __FILE__,
        ComponentPackageRole::Form,
        ExecutionStage::Form,
        "other_contract_form");
}

std::string baseMissionWithVehicleBody(const std::string& vehicle_body) {
    return R"json(
{
  "simulation": { "dt": 0.1, "duration": 0.1 },
  "outputs": { "enabled": false },
  "vehicles": [
    {
      "id": "vehicle",
)json" + vehicle_body +
           R"json(
    }
  ]
}
)json";
}

bool buildFailsWith(const std::string& mission, const std::string& fragment) {
    gnc::core::SimulationBuilder builder;
    test_support::require(builder.loadConfigString(mission),
                          "Form contract failure mission JSON did not parse.");
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
        registerContractFormTestTypes();

        const std::string valid_vehicle = R"json(
      "form": {
        "components": [
          { "type": "test.contract_6dof.form", "name": "dynamics", "config": {} }
        ]
      },
      "common": [],
      "input": [],
      "process": [
        {
          "type": "test.contract_6dof.force_moment_command",
          "name": "force_moment_command",
          "config": {
            "force_n": [1.0, 2.0, 3.0],
            "moment_nm": [0.1, 0.2, 0.3]
          }
        }
      ],
      "output": [],
      "interaction": {
        "components": [
          {
            "type": "test.contract_6dof.force_moment_interaction",
            "name": "interaction",
            "config": {}
          }
        ]
      }
)json";

        gnc::core::SimulationBuilder valid_builder;
        test_support::require(
            valid_builder.loadConfigString(baseMissionWithVehicleBody(valid_vehicle)),
            "Valid contract_6dof mission JSON did not parse.");
        auto& simulator = valid_builder.build();
        simulator.initialize();
        auto* truth_view =
            simulator.getRegistry().get<contract_6dof::ITruthView>(
                "vehicle.dynamics");
        auto* input_provider =
            simulator.getRegistry().get<contract_6dof::IInputProvider>(
                "vehicle.interaction");
        test_support::require(truth_view != nullptr && input_provider != nullptr,
                              "contract_6dof mission did not register required interfaces.");
        const auto input =
            input_provider->computeContract6DoFInput(truth_view->getContract6DoFTruth(),
                                                     0.0);
        test_support::requireVectorNear(input.force_n,
                                        gnc::math::Vector3(1.0, 2.0, 3.0),
                                        1.0e-12,
                                        "contract_6dof interaction lost force command.");
        test_support::requireVectorNear(input.moment_nm,
                                        gnc::math::Vector3(0.1, 0.2, 0.3),
                                        1.0e-12,
                                        "contract_6dof interaction lost moment command.");

        const std::string missing_family_vehicle = R"json(
      "form": {
        "components": [
          { "type": "test.form_without_family", "name": "dynamics", "config": {} }
        ]
      },
      "common": [],
      "input": [],
      "process": [],
      "output": [],
      "interaction": { "components": [] }
)json";
        test_support::require(
            buildFailsWith(baseMissionWithVehicleBody(missing_family_vehicle),
                           "must declare a non-empty form family"),
            "Form components without a form family must fail build.");

        const std::string mixed_family_vehicle = R"json(
      "form": {
        "components": [
          { "type": "test.contract_6dof.form", "name": "dynamics", "config": {} },
          { "type": "test.other_family_form", "name": "other_dynamics", "config": {} }
        ]
      },
      "common": [],
      "input": [],
      "process": [],
      "output": [],
      "interaction": { "components": [] }
)json";
        test_support::require(
            buildFailsWith(baseMissionWithVehicleBody(mixed_family_vehicle),
                           "selected form family for vehicle"),
            "A single vehicle must reject multiple form families.");

        const std::string mismatched_interaction_vehicle = R"json(
      "form": {
        "components": [
          { "type": "test.contract_6dof.form", "name": "dynamics", "config": {} }
        ]
      },
      "common": [],
      "input": [],
      "process": [],
      "output": [],
      "interaction": {
        "components": [
          {
            "type": "test_fixture.cartesian_3dof.acceleration_input",
            "name": "interaction",
            "config": { "acceleration_mps2": [0.0, 0.0, 0.0] }
          }
        ]
      }
)json";
        test_support::require(
            buildFailsWith(baseMissionWithVehicleBody(mismatched_interaction_vehicle),
                           "targets form family 'cartesian_3dof'"),
            "contract_6dof must reject interactions from a different form family.");

        std::cout << "form extension contract checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
