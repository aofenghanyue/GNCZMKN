/**
 * @file simulation_summary.hpp
 * @brief 仿真摘要报告生成器
 */
#pragma once

#include "gnc/common/logger.hpp"
#include "gnc/core/auto_data_logger.hpp"
#include "gnc/core/component_registry.hpp"

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <string>

namespace gnc::core {

class SimulationSummary {
public:
    struct SimInfo {
        double dt = 0.0;
        double duration = 0.0;
        double final_time = 0.0;
        int total_steps = 0;
        std::string termination_reason;
        double wall_clock_seconds = 0.0;
    };

    static void write(const std::string& output_dir,
                      const SimInfo& info,
                      const ComponentRegistry& registry,
                      const AutoDataLogger& logger) {
        if (output_dir.empty()) {
            return;
        }

        const std::string filepath = output_dir + "/summary.txt";
        std::ofstream file(filepath);
        if (!file.is_open()) {
            LOG_WARNING("SimulationSummary could not write summary file '{}' in output directory '{}'. Please check directory permissions.",
                        filepath, output_dir);
            return;
        }

        const auto now = std::chrono::system_clock::now();
        const auto time_value = std::chrono::system_clock::to_time_t(now);

        file << "========================================\n";
        file << "  GNC Simulation Summary Report\n";
        file << "========================================\n\n";
        file << "Generated: " << std::ctime(&time_value) << "\n";

        file << "--- Simulation Parameters ---\n";
        file << "  Time step (dt):      " << info.dt << " s\n";
        file << "  Configured duration: " << info.duration << " s\n";
        file << "  Actual final time:   " << info.final_time << " s\n";
        file << "  Total steps:         " << info.total_steps << "\n";
        file << "  Termination reason:  " << info.termination_reason << "\n";
        file << "  Wall clock time:     " << std::fixed << std::setprecision(3)
             << info.wall_clock_seconds << " s\n";
        file << "  Real-time ratio:     " << std::fixed << std::setprecision(1)
             << (info.wall_clock_seconds > 0.0 ? info.final_time / info.wall_clock_seconds : 0.0)
             << "x\n\n";

        file << "--- Components (" << registry.size() << ") ---\n";
        for (const auto& name : registry.getComponentNames()) {
            auto* component = registry.get<ComponentBase>(name);
            if (!component) {
                continue;
            }
            file << "  [" << name << "] freq=" << component->getExecutionFrequency() << " Hz\n";
        }
        file << "\n";

        file << "--- Data Recording ---\n";
        if (logger.isEnabled()) {
            file << "  Status:  ENABLED\n";
            file << "  Fields:  " << logger.getFieldCount() << "\n";
            file << "  Output:  " << logger.getOutputDir() << "\n";
        } else {
            file << "  Status:  DISABLED\n";
        }
        file << "\n";
        file << "========================================\n";

        LOG_INFO("SimulationSummary written to '{}'", filepath);
    }
};

} // namespace gnc::core
