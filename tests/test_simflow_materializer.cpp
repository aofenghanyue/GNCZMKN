#include "test_support.hpp"

#include "auto_registered_test_simflow_materializers.hpp"
#include "gnc/bootstrap/register_builtin_simflow_materializers.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/simflow/json_writer.hpp"
#include "gnc/simflow/simflow_config.hpp"
#include "gnc/simflow/simflow_materializer_registry.hpp"

#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <system_error>

namespace {

namespace fs = std::filesystem;

void writeFile(const fs::path& path, const std::string& text) {
    std::error_code ec;
    fs::create_directories(path.parent_path(), ec);
    test_support::require(!ec,
                          "Failed to create directory: " + path.generic_string());
    std::ofstream file(path, std::ios::binary);
    test_support::require(file.is_open(),
                          "Failed to write file: " + path.generic_string());
    file << text;
}

const char* baseMissionJson() {
    return R"json({
  "simulation": { "dt": 1.0, "duration": 0.0 },
  "environment": {},
  "vehicles": [
    {
      "id": "vehicle",
      "perturbation": {
        "type": "perturbation.static",
        "name": "perturbation",
        "config": { "inputs": {} }
      },
      "form": { "components": [] },
      "input": [],
      "process": [],
      "output": [],
      "interaction": { "components": [] }
    }
  ],
  "outputs": { "enabled": false, "directory": "old" }
})json";
}

} // namespace

int main() {
    try {
        const fs::path root = "user/outputs/test_simflow_materializer";
        std::error_code ec;
        fs::remove_all(root, ec);
        writeFile(root / "cases.csv",
                  "case_id,engine.temp_level,aero.drag_bias,label\n"
                  "hot,2,-0.03,high\n"
                  "cold,0,0.02,low\n");

        gnc::core::ConfigManager simflow_manager;
        test_support::require(simflow_manager.loadFromString(R"json({
  "base_mission": "user/outputs/test_simflow_materializer/base_mission.json",
  "materializer": {
    "type": "simflow.materializer.numeric_perturbation",
    "config": {}
  },
  "outputs": {
    "directory": "user/outputs/test_simflow_materializer/run",
    "case_directory": "case_{case_index}"
  }
})json"),
                              "SimFlow JSON did not parse.");
        const auto simflow_config =
            gnc::simflow::parseSimFlowConfig(simflow_manager.root());
        test_support::require(
            simflow_config.materializer_type ==
                "simflow.materializer.numeric_perturbation",
            "SimFlow materializer type parse mismatch.");
        test_support::require(simflow_config.output_directory ==
                                  "user/outputs/test_simflow_materializer/run",
                              "SimFlow output directory parse mismatch.");

        gnc::core::ConfigManager mission_manager;
        test_support::require(mission_manager.loadFromString(baseMissionJson()),
                              "Base mission JSON did not parse.");

        gnc::core::ConfigManager materializer_config_manager;
        test_support::require(materializer_config_manager.loadFromString(R"json({
  "case_source": {
    "mode": "matrix",
    "file": "user/outputs/test_simflow_materializer/cases.csv",
    "rows": [0, 1]
  },
  "vehicles": {
    "vehicle": {
      "inputs": ["engine.temp_level", "aero.drag_bias"]
    }
  }
})json"),
                              "Materializer config JSON did not parse.");

        auto& registry = gnc::simflow::SimFlowMaterializerRegistry::instance();
        registry.clearForTests();
        gnc::bootstrap::registerBuiltinSimFlowMaterializers(registry);
        gnc::build::registerAutoRegisteredTestSimFlowMaterializers(registry);

        test_support::require(
            registry.hasType("simflow.materializer.numeric_perturbation"),
            "Built-in numeric simflow materializer was not registered.");
        test_support::require(
            registry.hasType("test.simflow.project_materializer"),
            "Project simflow materializer was not registered.");

        auto materializer =
            registry.create("simflow.materializer.numeric_perturbation");
        gnc::simflow::SimFlowMaterializationContext context;
        context.base_mission = mission_manager.root();
        context.materializer_config = materializer_config_manager.root();
        context.output_directory = root / "run";
        context.case_directory_pattern = "case_{case_index}";
        context.repo_root = fs::current_path();

        const auto cases = materializer->materialize(context);
        test_support::require(cases.size() == 2,
                              "Numeric materializer should create two cases.");
        test_support::require(cases[0].case_id == "hot",
                              "Matrix case_id should become materialized case id.");

        const auto effective_text =
            gnc::simflow::writeJson(cases[0].effective_mission);
        test_support::require(effective_text.find("\"engine.temp_level\"") !=
                                  std::string::npos,
                              "Effective mission missing injected engine input.");
        test_support::require(effective_text.find("\"aero.drag_bias\"") !=
                                  std::string::npos,
                              "Effective mission missing injected aero input.");
        test_support::require(effective_text.find("case_000001") !=
                                  std::string::npos,
                              "Effective mission output directory should be case scoped.");

        auto project_materializer =
            registry.create("test.simflow.project_materializer");
        const auto project_cases = project_materializer->materialize(context);
        test_support::require(project_cases.size() == 1,
                              "Project materializer should create one case.");
        test_support::require(project_cases[0].case_id == "project_case",
                              "Project materializer case id mismatch.");

        std::cout << "simflow materializer checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
