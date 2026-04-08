#include "gnc/components/_builtin_register.hpp"
#include "gnc/core/simulation_builder.hpp"

#include <iostream>
#include <string>

int main() {
    constexpr const char* kConfig = R"({
        "simulation": {
            "dt": 0.1,
            "duration": 10.0,
            "stop_conditions": [
                {
                    "type": "component_field_below",
                    "component": "dynamics",
                    "field": "pos_z",
                    "value": 0.0,
                    "description": "ground_hit"
                }
            ]
        },
        "components": [
            {
                "type": "SimpleDynamics",
                "name": "dynamics",
                "config": {
                    "initial_position": [0.0, 0.0, 5.0],
                    "initial_velocity": [0.0, 0.0, -2.0]
                }
            }
        ]
    })";

    gnc::core::SimulationBuilder builder;
    if (!builder.loadConfigString(kConfig)) {
        std::cerr << "Failed to load inline stop condition config\n";
        return 1;
    }

    auto& simulator = builder.build();
    simulator.run();

    if (simulator.getTerminationReason() != "ground_hit") {
        std::cerr << "Expected stop condition to terminate the simulation via dynamics state field\n";
        return 1;
    }

    if (simulator.getCurrentTime() >= 10.0) {
        std::cerr << "Expected early termination before nominal duration\n";
        return 1;
    }

    std::cout << "Stop conditions support dynamics state fields\n";
    return 0;
}
