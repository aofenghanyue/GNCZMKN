#pragma once

#include "gnc/core/config_manager.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace gnc::runset {

struct VehicleCaseSourceConfig {
    std::string mode;
    std::string file;
    std::vector<size_t> rows;
};

struct RunSetConfig {
    std::string base_mission;
    std::string output_directory = "user/outputs";
    std::string case_directory = "case_{case_index}";
    std::unordered_map<std::string, VehicleCaseSourceConfig> vehicles;
};

inline RunSetConfig parseRunSetConfig(const gnc::core::ConfigNode& root) {
    RunSetConfig config;
    config.base_mission = root["base_mission"].asString();
    if (config.base_mission.empty()) {
        throw std::runtime_error("runset.base_mission is required.");
    }

    const auto& outputs = root["outputs"];
    if (!outputs.isNull()) {
        config.output_directory =
            outputs["directory"].asString(config.output_directory);
        config.case_directory =
            outputs["case_directory"].asString(config.case_directory);
    }

    const auto& vehicles = root["vehicles"];
    if (!vehicles.isObject()) {
        throw std::runtime_error("runset.vehicles must be an object.");
    }

    for (const auto& [vehicle_id, vehicle_node] : vehicles) {
        const auto& cases = vehicle_node["cases"];
        VehicleCaseSourceConfig source;
        source.mode = cases["mode"].asString();
        source.file = cases["file"].asString();
        const auto& rows = cases["rows"];
        if (!rows.isArray()) {
            throw std::runtime_error("runset.vehicles." + vehicle_id +
                                     ".cases.rows must be an array.");
        }
        for (size_t i = 0; i < rows.size(); ++i) {
            const double row_value = rows[i].asDouble();
            if (!rows[i].isNumber() || !std::isfinite(row_value) ||
                row_value < 0.0 || std::floor(row_value) != row_value ||
                row_value >
                    static_cast<double>(std::numeric_limits<size_t>::max())) {
                throw std::runtime_error("runset.vehicles." + vehicle_id +
                                         ".cases.rows entries must be non-negative integers.");
            }
            source.rows.push_back(static_cast<size_t>(row_value));
        }
        config.vehicles[vehicle_id] = std::move(source);
    }

    return config;
}

} // namespace gnc::runset
