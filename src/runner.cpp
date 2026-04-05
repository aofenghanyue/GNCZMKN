#include "gnc/common/logger.hpp"
#include "gnc/components/_builtin_register.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/simulation_builder.hpp"
#include "user_components_register.hpp"

#include <iostream>
#include <string>

using namespace gnc::core;

namespace {

constexpr const char* kDefaultConfig = "user/config/missions/default.json";

void printUsage(const char* program_name) {
    std::cout << "GNC Simulation Framework v2.0\n"
              << "Usage:\n"
              << "  " << program_name << "\n"
              << "  " << program_name << " <config.json>\n"
              << "  " << program_name << " --list-components\n"
              << "  " << program_name << " --help\n"
              << "Default config: " << kDefaultConfig << "\n";
}

void listComponents() {
    auto types = ComponentFactory::instance().getRegisteredTypes();
    std::cout << "Registered component types (" << types.size() << "):\n";
    for (const auto& type : types) {
        std::cout << "  - " << type << "\n";
    }
}

}

int main(int argc, char* argv[]) {
    std::string config_file = kDefaultConfig;

    if (argc > 1) {
        const std::string arg = argv[1];
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        }
        if (arg == "--list-components") {
            listComponents();
            return 0;
        }
        config_file = arg;
    }

    LOG_INFO("=== GNC Simulation Framework v2.0 ===");
    LOG_INFO("Config: {}", config_file);

    try {
        SimulationBuilder builder;
        if (!builder.loadConfig(config_file)) {
            LOG_ERROR("Failed to load config file '{}' in runner. Please check the file path and JSON syntax.", config_file);
            return 1;
        }

        auto& simulator = builder.build();
        simulator.run();

        LOG_INFO("=== Simulation Completed ===");
        return 0;
    } catch (const std::exception& e) {
        LOG_ERROR("Simulation failed in runner: {}. Please review the diagnostics above and fix the reported issue.", e.what());
        return 1;
    }
}
