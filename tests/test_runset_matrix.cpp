#include "test_support.hpp"

#include "gnc/core/config_manager.hpp"
#include "gnc/runset/config_json_writer.hpp"
#include "gnc/runset/matrix_case_source.hpp"
#include "gnc/runset/mission_overlay.hpp"
#include "gnc/runset/runset_config.hpp"
#include "gnc/runset/runset_runner.hpp"

#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <system_error>
#include <unordered_map>

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

std::string readText(const fs::path& path) {
    std::ifstream file(path, std::ios::binary);
    test_support::require(file.is_open(),
                          "Failed to read file: " + path.generic_string());
    std::ostringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

} // namespace

int main() {
    try {
        const char* mission = R"json(
{
  "simulation": { "dt": 1.0, "duration": 0.0 },
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
  "outputs": { "enabled": false }
}
)json";

        gnc::core::ConfigManager manager;
        test_support::require(manager.loadFromString(mission),
                              "Base mission JSON did not parse.");

        std::unordered_map<std::string, double> inputs{
            {"engine.temp_level", 2.0},
            {"aero.drag_bias", -0.03}
        };
        auto overlaid =
            gnc::runset::overlayVehiclePerturbationInputs(manager.root(),
                                                          "vehicle",
                                                          inputs);
        const auto text = gnc::runset::writeJson(overlaid);
        test_support::require(text.find("\"engine.temp_level\"") != std::string::npos,
                              "Overlay did not write engine.temp_level.");
        test_support::require(text.find("-0.03") != std::string::npos,
                              "Overlay did not write aero.drag_bias.");

        const std::string matrix =
            "case_id,engine.temp_level,aero.drag_bias\n"
            "hot,2,-0.03\n"
            "cold,0,0.02\n";
        const auto cases = gnc::runset::parseMatrixCases(matrix, {1});
        test_support::require(cases.size() == 1,
                              "Row selection should return one matrix case.");
        test_support::require(cases[0].inputs.at("engine.temp_level") == 0.0,
                              "Selected row engine.temp_level mismatch.");
        test_support::require(cases[0].inputs.at("aero.drag_bias") == 0.02,
                              "Selected row aero.drag_bias mismatch.");

        const std::string malformed_matrix =
            "case_id,engine.temp_level\n"
            "bad,1abc\n";
        bool malformed_failed = false;
        try {
            (void)gnc::runset::parseMatrixCases(malformed_matrix, {0});
        } catch (const std::exception&) {
            malformed_failed = true;
        }
        test_support::require(malformed_failed,
                              "Matrix parser should reject numeric cells with trailing text.");

        const std::string whitespace_matrix =
            "case_id,engine.temp_level\n"
            "ok,1.5 \t\n";
        const auto whitespace_cases =
            gnc::runset::parseMatrixCases(whitespace_matrix, {0});
        test_support::require(whitespace_cases[0].inputs.at("engine.temp_level") == 1.5,
                              "Matrix parser should allow trailing whitespace.");

        const char* fractional_rows_runset = R"json(
{
  "base_mission": "base.json",
  "vehicles": {
    "vehicle": {
      "cases": {
        "mode": "matrix",
        "file": "matrix.csv",
        "rows": [0.5]
      }
    }
  }
}
)json";
        gnc::core::ConfigManager fractional_rows_manager;
        test_support::require(
            fractional_rows_manager.loadFromString(fractional_rows_runset),
            "Fractional rows RunSet JSON did not parse.");
        bool fractional_rows_failed = false;
        try {
            (void)gnc::runset::parseRunSetConfig(fractional_rows_manager.root());
        } catch (const std::exception&) {
            fractional_rows_failed = true;
        }
        test_support::require(fractional_rows_failed,
                              "RunSet rows must reject fractional row indices.");

        const fs::path root = fs::path("user/outputs/test_runset_matrix");
        std::error_code ec;
        fs::remove_all(root, ec);

        writeFile(root / "base_mission.json", R"json(
{
  "simulation": { "dt": 1.0, "duration": 0.0 },
  "environment": {},
  "vehicles": [
    {
      "id": "vehicle",
      "perturbation": {
        "type": "perturbation.static",
        "name": "perturbation",
        "config": {
          "inputs": {},
          "enum_maps": {
            "engine.temp_level": {
              "0": "cold.txt",
              "2": "hot.txt"
            }
          }
        }
      },
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
      "input": [],
      "process": [],
      "output": [],
      "interaction": {
        "components": [
          {
            "type": "interaction.cartesian_3dof.direct_accel",
            "name": "interaction",
            "config": { "acceleration_mps2": [0.0, 0.0, 0.0] }
          }
        ]
      }
    }
  ],
  "outputs": {
    "enabled": true,
    "directory": "user/outputs/test_runset_matrix/shared_outputs",
    "session_name": "shared_data"
  }
}
)json");

        writeFile(root / "matrix.csv",
                  "case_id,engine.temp_level,aero.drag_bias\n"
                  "hot,2,-0.03\n"
                  "cold,0,0.02\n");

        writeFile(root / "runset.json", R"json(
{
  "base_mission": "user/outputs/test_runset_matrix/base_mission.json",
  "vehicles": {
    "vehicle": {
      "cases": {
        "mode": "matrix",
        "file": "user/outputs/test_runset_matrix/matrix.csv",
        "rows": [0, 1]
      }
    }
  },
  "outputs": {
    "directory": "user/outputs/test_runset_matrix/run",
    "case_directory": "case_{case_index}"
  }
}
)json");

        test_support::registerBuiltinComponentTypes();
        gnc::runset::RunSetRunner runner;
        runner.runSerial((root / "runset.json").string());

        const auto effective =
            readText(root / "run" / "case_000001" / "effective_mission.json");
        test_support::require(effective.find("\"engine.temp_level\"") !=
                                  std::string::npos,
                              "Effective mission missing perturbation input.");
        test_support::require(
            effective.find(
                "\"directory\": \"user/outputs/test_runset_matrix/run/case_000001\"") !=
                std::string::npos,
            "Effective mission should route case 1 outputs into its case directory.");
        const auto inputs_text =
            readText(root / "run" / "case_000001" / "perturbation_inputs.json");
        test_support::require(inputs_text.find("\"aero.drag_bias\"") !=
                                  std::string::npos,
                              "Perturbation inputs file missing aero.drag_bias.");
        test_support::require(
            fs::exists(root / "run" / "case_000001" / "shared_data.csv"),
            "Case 1 CSV should be written under case_000001.");
        test_support::require(
            fs::exists(root / "run" / "case_000001" / "summary.txt"),
            "Case 1 summary should be written under case_000001.");
        test_support::require(
            fs::exists(root / "run" / "case_000002" / "shared_data.csv"),
            "Case 2 CSV should be written under case_000002.");
        test_support::require(
            fs::exists(root / "run" / "case_000002" / "summary.txt"),
            "Case 2 summary should be written under case_000002.");

        std::cout << "runset overlay checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
