#include "gnc/components/_builtin_register.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/core/simulation_builder.hpp"
#include "gnc/interfaces/state/i_position_provider.hpp"

#include <iostream>
#include <stdexcept>
#include <string>

class InjectionOnlyConsumer : public gnc::core::ComponentBase {
public:
    InjectionOnlyConsumer() : ComponentBase("InjectionOnlyConsumer") {}

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(gnc::core::bind(position_provider_, "tracker"));
    }

    void update(double) override {}

private:
    gnc::interfaces::IPositionProvider* position_provider_ = nullptr;
};

GNC_REGISTER_COMPONENT(InjectionOnlyConsumer, gnc::core::ComponentBase)

int main() {
    constexpr const char* kConfig = R"({
        "simulation": {
            "dt": 0.1,
            "duration": 1.0
        },
        "components": [
            {
                "type": "InjectionOnlyConsumer",
                "name": "consumer"
            }
        ]
    })";

    gnc::core::SimulationBuilder builder;
    if (!builder.loadConfigString(kConfig)) {
        std::cerr << "Failed to load inline dependency preflight config\n";
        return 1;
    }

    try {
        builder.build();
        std::cerr << "Expected build to fail when a required bind(...) target is missing\n";
        return 1;
    } catch (const std::runtime_error&) {
        std::cout << "Implicit dependency preflight works\n";
        return 0;
    }
}
