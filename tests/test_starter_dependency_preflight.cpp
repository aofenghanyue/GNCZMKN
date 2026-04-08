#include "gnc/components/_builtin_register.hpp"
#include "gnc/core/simulation_builder.hpp"

#include <iostream>
#include <stdexcept>

namespace {

bool expectBuildFailure(const char* config_json) {
    gnc::core::SimulationBuilder builder;
    if (!builder.loadConfigString(config_json)) {
        std::cerr << "Failed to load inline config\n";
        return false;
    }

    try {
        builder.build();
        return false;
    } catch (const std::runtime_error&) {
        return true;
    }
}

} // namespace

int main() {
    constexpr const char* kMissingDynamicsForNav = R"({
        "simulation": {
            "dt": 0.1,
            "duration": 1.0
        },
        "components": [
            {
                "type": "SimpleNavigation",
                "name": "nav"
            }
        ]
    })";

    constexpr const char* kMissingDynamicsForTruth = R"({
        "simulation": {
            "dt": 0.1,
            "duration": 1.0
        },
        "components": [
            {
                "type": "TruthState",
                "name": "truth"
            }
        ]
    })";

    constexpr const char* kMissingAssemblyDependencies = R"({
        "simulation": {
            "dt": 0.1,
            "duration": 1.0
        },
        "components": [
            {
                "type": "Dynamics3DOF_SphericalEarth",
                "name": "dynamics"
            }
        ]
    })";

    if (!expectBuildFailure(kMissingDynamicsForNav)) {
        std::cerr << "SimpleNavigation should fail build preflight when dynamics providers are missing\n";
        return 1;
    }

    if (!expectBuildFailure(kMissingDynamicsForTruth)) {
        std::cerr << "TruthState should fail build preflight when dynamics providers are missing\n";
        return 1;
    }

    if (!expectBuildFailure(kMissingAssemblyDependencies)) {
        std::cerr << "Dynamics3DOF_SphericalEarth should fail build preflight when assembly dependencies are missing\n";
        return 1;
    }

    std::cout << "Starter dependency preflight works without explicit declarers\n";
    return 0;
}
