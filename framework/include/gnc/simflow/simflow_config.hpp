#pragma once

#include "gnc/core/config_manager.hpp"

#include <stdexcept>
#include <string>

namespace gnc::simflow {

struct SimFlowConfig {
    std::string base_mission;
    std::string materializer_type;
    gnc::core::ConfigNode materializer_config;
    std::string output_directory = "user/outputs";
    std::string case_directory = "case_{case_index}";
};

inline SimFlowConfig parseSimFlowConfig(const gnc::core::ConfigNode& root) {
    SimFlowConfig config;
    config.base_mission = root["base_mission"].asString();
    if (config.base_mission.empty()) {
        throw std::runtime_error("simflow.base_mission is required.");
    }

    const auto& materializer = root["materializer"];
    if (!materializer.isObject()) {
        throw std::runtime_error("simflow.materializer must be an object.");
    }
    config.materializer_type = materializer["type"].asString();
    if (config.materializer_type.empty()) {
        throw std::runtime_error("simflow.materializer.type is required.");
    }
    config.materializer_config = materializer["config"];
    if (config.materializer_config.isNull()) {
        config.materializer_config = gnc::core::ConfigNode::makeObject();
    }
    if (!config.materializer_config.isObject()) {
        throw std::runtime_error("simflow.materializer.config must be an object.");
    }

    const auto& outputs = root["outputs"];
    if (!outputs.isNull()) {
        if (!outputs.isObject()) {
            throw std::runtime_error("simflow.outputs must be an object.");
        }
        config.output_directory =
            outputs["directory"].asString(config.output_directory);
        config.case_directory =
            outputs["case_directory"].asString(config.case_directory);
    }
    if (config.output_directory.empty()) {
        throw std::runtime_error("simflow.outputs.directory must not be empty.");
    }
    if (config.case_directory.empty()) {
        throw std::runtime_error("simflow.outputs.case_directory must not be empty.");
    }
    return config;
}

} // namespace gnc::simflow
