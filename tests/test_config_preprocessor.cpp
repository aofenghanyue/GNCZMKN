#include "test_support.hpp"

#include "gnc/core/config_manager.hpp"
#include "gnc/core/simulation_builder.hpp"

#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <system_error>

namespace {

namespace fs = std::filesystem;

fs::path repoRoot() {
    return fs::path(__FILE__).lexically_normal().parent_path().parent_path();
}

std::string normalizePath(const fs::path& path) {
    return path.lexically_normal().generic_string();
}

void writeFile(const fs::path& path, const std::string& text) {
    std::error_code ec;
    fs::create_directories(path.parent_path(), ec);
    test_support::require(!ec,
                          "Failed to create directory for " + normalizePath(path));

    std::ofstream stream(path, std::ios::binary);
    test_support::require(stream.is_open(),
                          "Failed to write file: " + normalizePath(path));
    stream << text;
}

struct ScratchDirs {
    explicit ScratchDirs(fs::path root)
        : scratch(root / "user/outputs/test_config_preprocessor"),
          project(root / "user/example_05_ideal_3dof_geographic_baseline/config/preprocessor_test"),
          user_data(root / "user/data/preprocessor_test") {
        cleanup();
        std::error_code ec;
        fs::create_directories(scratch, ec);
        test_support::require(!ec, "Failed to create config preprocessor scratch dir.");
        fs::create_directories(project, ec);
        test_support::require(!ec, "Failed to create project preprocessor scratch dir.");
        fs::create_directories(user_data, ec);
        test_support::require(!ec, "Failed to create user-data preprocessor scratch dir.");
    }

    ~ScratchDirs() { cleanup(); }

    void cleanup() const {
        std::error_code ec;
        fs::remove_all(scratch, ec);
        ec.clear();
        fs::remove_all(project, ec);
        ec.clear();
        fs::remove_all(user_data, ec);
    }

    fs::path scratch;
    fs::path project;
    fs::path user_data;
};

void requireLoadFailure(const fs::path& path, const std::string& message) {
    gnc::core::ConfigManager manager;
    test_support::require(!manager.loadFromFile(path.string()), message);
}

void requireExpandedLegacySchemaFails(const fs::path& root_file) {
    gnc::core::SimulationBuilder builder;
    test_support::require(builder.loadConfig(root_file.string()),
                          "Legacy-schema include mission should parse and preprocess.");

    bool failed = false;
    try {
        builder.build();
    } catch (const std::exception&) {
        failed = true;
    }

    bool reported_legacy_block = false;
    for (const auto& error : builder.getBuildErrors()) {
        if (error.find("Legacy root-level mission blocks") != std::string::npos) {
            reported_legacy_block = true;
        }
    }

    test_support::require(failed,
                          "Expanded legacy schema should fail during mission build.");
    test_support::require(
        reported_legacy_block,
        "Expanded legacy schema did not report the legacy root component block.");
}

} // namespace

int main() {
    try {
        test_support::registerBuiltinComponentTypes();

        const fs::path root = repoRoot();
        ScratchDirs dirs(root);

        writeFile(dirs.scratch / "base_a.json",
                  R"json({
  "node": {
    "x": 1.0,
    "nested": { "a": 1.0 },
    "arr": [1.0],
    "from_a": true
  }
}
)json");
        writeFile(dirs.scratch / "base_b.json",
                  R"json({
  "node": {
    "y": 2.0,
    "nested": { "b": 2.0 },
    "arr": [2.0]
  }
}
)json");
        writeFile(dirs.scratch / "merged.json",
                  R"json({
  "$include": ["base_a.json", "base_b.json"],
  "node": {
    "x": 3.0,
    "nested": { "a": 4.0 }
  }
}
)json");

        gnc::core::ConfigManager merge_manager;
        test_support::require(
            merge_manager.loadFromFile((dirs.scratch / "merged.json").string()),
            "Relative include merge config failed to load.");
        const auto& merged_node = merge_manager.root()["node"];
        test_support::requireNear(merged_node["x"].asDouble(), 3.0, 1e-12,
                                  "Local object field did not override include.");
        test_support::requireNear(merged_node["y"].asDouble(), 2.0, 1e-12,
                                  "Later include field was not preserved.");
        test_support::requireNear(merged_node["nested"]["a"].asDouble(),
                                  4.0,
                                  1e-12,
                                  "Local nested field did not override include.");
        test_support::requireNear(merged_node["nested"]["b"].asDouble(),
                                  2.0,
                                  1e-12,
                                  "Deep merge lost nested include field.");
        test_support::require(merged_node["arr"].size() == 1 &&
                                  merged_node["arr"][0].asDouble() == 2.0,
                              "Arrays must be replaced wholesale by later includes.");

        writeFile(dirs.scratch / "repo_fragment.json",
                  R"json({ "repo_value": 7.0 })json");
        writeFile(dirs.user_data / "defaults.json",
                  R"json({ "data_value": 8.0 })json");
        writeFile(dirs.project / "project_fragment.json",
                  R"json({ "project_value": 9.0 })json");
        writeFile(dirs.project / "uri_root.json",
                  R"json({
  "$include": [
    "repo://user/outputs/test_config_preprocessor/repo_fragment.json",
    "user-data://preprocessor_test/defaults.json",
    "project://config/preprocessor_test/project_fragment.json"
  ],
  "project_value": 10.0
}
)json");

        gnc::core::ConfigManager uri_manager;
        test_support::require(
            uri_manager.loadFromFile((dirs.project / "uri_root.json").string()),
            "URI include config failed to load.");
        test_support::requireNear(uri_manager.root()["repo_value"].asDouble(),
                                  7.0,
                                  1e-12,
                                  "repo:// include did not resolve from repo root.");
        test_support::requireNear(uri_manager.root()["data_value"].asDouble(),
                                  8.0,
                                  1e-12,
                                  "user-data:// include did not resolve from user/data.");
        test_support::requireNear(uri_manager.root()["project_value"].asDouble(),
                                  10.0,
                                  1e-12,
                                  "project:// include or local override failed.");

        writeFile(dirs.scratch / "cycle_a.json",
                  R"json({ "$include": "cycle_b.json" })json");
        writeFile(dirs.scratch / "cycle_b.json",
                  R"json({ "$include": "cycle_a.json" })json");
        requireLoadFailure(dirs.scratch / "cycle_a.json",
                           "Include cycle should fail during preprocessing.");

        writeFile(dirs.scratch / "missing.json",
                  R"json({ "$include": "does_not_exist.json" })json");
        requireLoadFailure(dirs.scratch / "missing.json",
                           "Missing include file should fail during preprocessing.");

        writeFile(dirs.scratch / "invalid_include_type.json",
                  R"json({ "$include": 42 })json");
        requireLoadFailure(dirs.scratch / "invalid_include_type.json",
                           "Invalid $include type should fail during preprocessing.");

        gnc::core::ConfigManager string_manager;
        test_support::require(
            !string_manager.loadFromString(R"json({ "$include": "base_a.json" })json"),
            "loadFromString must reject filesystem $include directives.");

        writeFile(dirs.scratch / "legacy_root_fragment.json",
                  R"json({ "components": [] })json");
        writeFile(dirs.scratch / "legacy_expanded_mission.json",
                  R"json({
  "$include": "legacy_root_fragment.json",
  "simulation": { "dt": 0.1, "duration": 0.1, "integrator": "rk4" },
  "environment": {},
  "vehicles": [
    {
      "id": "vehicle",
      "form": {
        "components": [
          {
            "type": "form.cartesian_3dof.point_mass",
            "name": "dynamics",
            "config": {
              "initial_position": [0.0, 0.0, 1000.0],
              "initial_velocity": [0.0, 0.0, 0.0]
            }
          }
        ]
      },
      "common": [],
      "input": [],
      "process": [],
      "output": [],
      "interaction": {
        "components": [
          {
            "type": "test_fixture.cartesian_3dof.acceleration_input",
            "name": "interaction",
            "config": {
              "acceleration_mps2": [0.0, 0.0, -9.81]
            }
          }
        ]
      }
    }
  ],
  "outputs": { "enabled": false }
}
)json");
        requireExpandedLegacySchemaFails(dirs.scratch / "legacy_expanded_mission.json");

        std::cout << "config preprocessor checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
