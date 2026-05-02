#include "test_support.hpp"

#include "gnc/core/component_factory.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/core/simulation_builder.hpp"
#include "gnc/perturbation/components/static_perturbation.hpp"
#include "gnc/perturbation/interfaces/i_perturbation_provider.hpp"

#include <exception>
#include <iostream>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace {

std::vector<std::string> g_trace;

class TraceComponent : public gnc::core::ComponentBase {
public:
    explicit TraceComponent(std::string label)
        : ComponentBase("TraceComponent"), label_(std::move(label)) {}

    void initialize() override { g_trace.push_back("init:" + label_); }
    void update(double) override { g_trace.push_back("update:" + label_); }

private:
    std::string label_;
};

class PerturbationProbe final
    : public TraceComponent,
      public gnc::perturbation::IPerturbationProvider,
      public gnc::perturbation::IPerturbationSnapshot {
public:
    PerturbationProbe() : TraceComponent("perturbation") {}

    bool has(const std::string& key) const override { return key == "probe.value"; }

    double getNumber(const std::string& key, double fallback) const override {
        return key == "probe.value" ? 42.0 : fallback;
    }

    std::string getString(const std::string&, const std::string& fallback) const override {
        return fallback;
    }

    std::vector<double> getVector(const std::string&) const override { return {}; }

    std::map<std::string, gnc::perturbation::PerturbationValue>
    snapshotResolvedState() const override {
        gnc::perturbation::PerturbationValue value;
        value.type = gnc::perturbation::PerturbationValue::Type::Number;
        value.number = 42.0;
        return {{"probe.value", value}};
    }
};

class InputProbe final : public TraceComponent {
public:
    InputProbe() : TraceComponent("input") {}
};

class FormProbe final : public TraceComponent {
public:
    FormProbe() : TraceComponent("form") {}
};

class NonPerturbationProbe final : public TraceComponent {
public:
    NonPerturbationProbe() : TraceComponent("non_perturbation") {}
};

void registerTypes() {
    using namespace gnc::core;
    auto& factory = ComponentFactory::instance();
    factory.registerType<PerturbationProbe,
                         gnc::perturbation::IPerturbationProvider,
                         gnc::perturbation::IPerturbationSnapshot>(
        "test.perturbation_probe",
        ComponentCategory::Project,
        __FILE__,
        ComponentPackageRole::Perturbation,
        ExecutionStage::Perturbation);
    factory.registerType<InputProbe>(
        "test.perturbation_input_probe",
        ComponentCategory::Project,
        __FILE__,
        ComponentPackageRole::VehicleInput,
        ExecutionStage::VehicleInput);
    factory.registerType<FormProbe>(
        "test.perturbation_form_probe",
        ComponentCategory::Project,
        __FILE__,
        ComponentPackageRole::Form,
        ExecutionStage::Form,
        "test_form");
    factory.registerType<NonPerturbationProbe>(
        "test.non_perturbation_probe",
        ComponentCategory::Project,
        __FILE__,
        ComponentPackageRole::Perturbation,
        ExecutionStage::Perturbation);
}

} // namespace

int main() {
    try {
        test_support::registerBuiltinComponentTypes();
        registerTypes();
        g_trace.clear();

        const char* mission = R"json(
{
  "simulation": { "dt": 1.0, "duration": 0.0 },
  "environment": {},
  "vehicles": [
    {
      "id": "vehicle",
      "perturbation": {
        "type": "test.perturbation_probe",
        "name": "perturbation",
        "config": {}
      },
      "form": {
        "components": [
          { "type": "test.perturbation_form_probe", "name": "dynamics", "config": {} }
        ]
      },
      "input": [
        { "type": "test.perturbation_input_probe", "name": "input", "config": {} }
      ],
      "process": [],
      "output": [],
      "interaction": { "components": [] }
    }
  ],
  "outputs": { "enabled": false }
}
)json";

        gnc::core::SimulationBuilder builder;
        test_support::require(builder.loadConfigString(mission),
                              "Perturbation lifecycle mission JSON did not parse.");
        builder.build().run();

        test_support::require(g_trace.size() >= 4,
                              "Expected perturbation lifecycle trace entries.");
        test_support::require(g_trace[0] == "init:perturbation",
                              "Perturbation should initialize before other vehicle components.");
        test_support::require(g_trace[3] == "update:perturbation",
                              "Perturbation should update before vehicle input at t0.");

        const char* invalid_mission = R"json(
{
  "simulation": { "dt": 1.0, "duration": 0.0 },
  "vehicles": [
    {
      "id": "vehicle",
      "perturbation": {
        "type": "test.non_perturbation_probe",
        "name": "perturbation",
        "config": {}
      },
      "form": {
        "components": [
          { "type": "test.perturbation_form_probe", "name": "dynamics", "config": {} }
        ]
      },
      "input": [],
      "process": [],
      "output": [],
      "interaction": { "components": [] }
    }
  ],
  "outputs": { "enabled": false }
}
)json";

        gnc::core::SimulationBuilder invalid_builder;
        test_support::require(invalid_builder.loadConfigString(invalid_mission),
                              "Invalid perturbation mission JSON did not parse.");
        bool invalid_failed = false;
        try {
            invalid_builder.build();
        } catch (const std::exception&) {
            invalid_failed = true;
        }
        test_support::require(invalid_failed,
                              "Perturbation placement should require IPerturbationProvider.");

        const char* invalid_static_config_mission = R"json(
{
  "simulation": { "dt": 1.0, "duration": 0.0 },
  "vehicles": [
    {
      "id": "vehicle",
      "perturbation": {
        "type": "perturbation.static",
        "name": "perturbation",
        "config": {
          "input": {
            "probe.value": 1.0
          }
        }
      },
      "form": {
        "components": [
          { "type": "test.perturbation_form_probe", "name": "dynamics", "config": {} }
        ]
      },
      "input": [],
      "process": [],
      "output": [],
      "interaction": { "components": [] }
    }
  ],
  "outputs": { "enabled": false }
}
)json";

        gnc::core::SimulationBuilder invalid_static_config_builder;
        test_support::require(
            invalid_static_config_builder.loadConfigString(invalid_static_config_mission),
            "Invalid static perturbation config mission JSON did not parse.");
        bool invalid_static_config_failed = false;
        try {
            invalid_static_config_builder.build();
        } catch (const std::exception&) {
            invalid_static_config_failed = true;
        }
        test_support::require(
            invalid_static_config_failed,
            "perturbation.static should reject unrecognized top-level config keys.");

        gnc::core::ConfigManager invalid_static_direct_config;
        test_support::require(
            invalid_static_direct_config.loadFromString(
                R"json({ "input": { "probe.value": 1.0 } })json"),
            "Invalid static perturbation direct config JSON did not parse.");
        gnc::perturbation::StaticPerturbation static_perturbation;
        bool invalid_static_direct_config_failed = false;
        try {
            static_perturbation.configure(invalid_static_direct_config.root(),
                                          "perturbation.static.config");
        } catch (const std::exception&) {
            invalid_static_direct_config_failed = true;
        }
        test_support::require(
            invalid_static_direct_config_failed,
            "StaticPerturbation::configure should reject unrecognized top-level keys.");

        std::cout << "perturbation contract checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
