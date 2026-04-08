#include "gnc/components/_builtin_register.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/core/simulation_builder.hpp"
#include "gnc/interfaces/state/i_position_provider.hpp"

#include <iostream>

class CountingInjectionConsumer : public gnc::core::ComponentBase {
public:
    CountingInjectionConsumer() : ComponentBase("CountingInjectionConsumer") {}

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        ++inject_calls_;
        registry.bindAll(gnc::core::bind(position_provider_, "dynamics"));
    }

    void update(double) override {}

    static void reset() {
        inject_calls_ = 0;
    }

    static int injectCalls() {
        return inject_calls_;
    }

private:
    inline static int inject_calls_ = 0;
    gnc::interfaces::IPositionProvider* position_provider_ = nullptr;
};

GNC_REGISTER_COMPONENT(CountingInjectionConsumer, gnc::core::ComponentBase)

int main() {
    CountingInjectionConsumer::reset();

    constexpr const char* kConfig = R"({
        "simulation": {
            "dt": 0.1,
            "duration": 0.5
        },
        "components": [
            {
                "type": "SimpleDynamics",
                "name": "dynamics"
            },
            {
                "type": "CountingInjectionConsumer",
                "name": "consumer"
            }
        ]
    })";

    gnc::core::SimulationBuilder builder;
    if (!builder.loadConfigString(kConfig)) {
        std::cerr << "Failed to load inline dependency lifecycle config\n";
        return 1;
    }

    auto& simulator = builder.build();
    if (CountingInjectionConsumer::injectCalls() != 1) {
        std::cerr << "Expected dependency preflight to inject exactly once during build\n";
        return 1;
    }

    simulator.run();
    if (CountingInjectionConsumer::injectCalls() != 1) {
        std::cerr << "Expected simulator initialization to skip reinjecting preflighted dependencies\n";
        return 1;
    }

    std::cout << "Dependency injection lifecycle works\n";
    return 0;
}
