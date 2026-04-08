#include "gnc/components/_builtin_register.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/dependency_validator.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/core/simulation_builder.hpp"
#include "gnc/interfaces/state/i_position_provider.hpp"

#include <iostream>
#include <stdexcept>

class DeclaredButMismatchedConsumer : public gnc::core::ComponentBase,
                                      public gnc::core::IDependencyDeclarer {
public:
    DeclaredButMismatchedConsumer() : ComponentBase("DeclaredButMismatchedConsumer") {}

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(gnc::core::bind(position_provider_, "tracer"));
    }

    std::vector<gnc::core::DependencyDeclaration> getDependencies() const override {
        return {
            gnc::core::requireDependency<gnc::interfaces::IPositionProvider>(
                "tracker", "a tracker position provider")
        };
    }

    void update(double) override {}

private:
    gnc::interfaces::IPositionProvider* position_provider_ = nullptr;
};

GNC_REGISTER_COMPONENT(DeclaredButMismatchedConsumer, gnc::core::ComponentBase)

int main() {
    constexpr const char* kConfig = R"({
        "simulation": {
            "dt": 0.1,
            "duration": 1.0
        },
        "components": [
            {
                "type": "SimpleDynamics",
                "name": "tracker"
            },
            {
                "type": "DeclaredButMismatchedConsumer",
                "name": "consumer"
            }
        ]
    })";

    gnc::core::SimulationBuilder builder;
    if (!builder.loadConfigString(kConfig)) {
        std::cerr << "Failed to load inline config for declared dependency preflight test\n";
        return 1;
    }

    try {
        builder.build();
        std::cerr << "Expected build to fail when injectDependencies() drifts from declared dependencies\n";
        return 1;
    } catch (const std::runtime_error&) {
        std::cout << "Declared dependency preflight works\n";
        return 0;
    }
}
