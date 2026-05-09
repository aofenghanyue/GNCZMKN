#pragma once

#include "gnc/core/config_manager.hpp"

#include <filesystem>
#include <string>
#include <vector>

namespace gnc::simflow {

struct SimFlowMaterializationContext {
    gnc::core::ConfigNode simflow_config;
    gnc::core::ConfigNode materializer_config;
    gnc::core::ConfigNode base_mission;
    std::filesystem::path simflow_file;
    std::filesystem::path repo_root;
    std::filesystem::path project_root;
    std::filesystem::path output_directory;
    std::string case_directory_pattern = "case_{case_index}";
};

struct MaterializedCase {
    std::string case_id;
    gnc::core::ConfigNode effective_mission;
};

class ISimFlowMaterializer {
public:
    virtual ~ISimFlowMaterializer() = default;

    virtual std::vector<MaterializedCase> materialize(
        const SimFlowMaterializationContext& context) = 0;
};

} // namespace gnc::simflow
