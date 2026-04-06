#include "gnc/common/logger.hpp"
#include "gnc/components/_builtin_register.hpp"
#include "gnc/core/simulation_builder.hpp"

#include "cavh_aerodynamics.hpp"
#include "cavh_mass.hpp"
#include "cavh_programmed_aoa.hpp"

#include <string>

int main(int argc, char* argv[]) {
    const std::string config_file = argc > 1
        ? argv[1]
        : "examples/03_cavh_3dof/cavh_mission.json";

    gnc::core::SimulationBuilder builder;
    if (!builder.loadConfig(config_file)) {
        LOG_ERROR("Failed to load CAV-H config '{}'. Please check the file path and JSON syntax.", config_file);
        return 1;
    }

    try {
        auto& simulator = builder.build();
        simulator.run();
        return 0;
    } catch (const std::exception& e) {
        LOG_ERROR("CAV-H example failed: {}. Please review the diagnostics above and fix the reported issue.", e.what());
        return 1;
    }
}
