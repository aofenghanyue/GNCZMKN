#include "test_support.hpp"

#include "gnc/bootstrap/register_builtin_simflow_materializers.hpp"
#include "gnc/simflow/path_utils.hpp"
#include "gnc/simflow/simflow_materializer_registry.hpp"
#include "gnc/simflow/simflow_runner.hpp"

#include <cstdlib>
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

std::string readText(const fs::path& path) {
    std::ifstream file(path, std::ios::binary);
    test_support::require(file.is_open(),
                          "Failed to read file: " + path.generic_string());
    std::ostringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

fs::path gncSimPath(const char* argv0) {
    fs::path executable = fs::absolute(fs::path(argv0)).parent_path() / "gnc_sim";
#ifdef _WIN32
    executable += ".exe";
#endif
    return executable;
}

std::string baseMissionJson() {
    return R"json({
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
      "common": [
        { "type": "vehicle.common.aero_assets_3dof.zero", "name": "aero_assets", "config": {} }
      ],
      "input": [],
      "process": [],
      "output": [
        { "type": "vehicle.output.mass_3dof.constant", "name": "mass", "config": { "mass_kg": 100.0 } },
        { "type": "vehicle.output.propulsion_3dof.zero", "name": "propulsion", "config": {} },
        { "type": "vehicle.output.actuator_3dof.ideal", "name": "actuator", "config": {} },
        { "type": "vehicle.output.aerodynamics_3dof.zero", "name": "aero", "config": {} }
      ],
      "interaction": {
        "components": [
          {
            "type": "interaction.cartesian_3dof.standard",
            "name": "interaction",
            "config": { "acceleration_mps2": [0.0, 0.0, 0.0] }
          }
        ]
      }
    }
  ],
  "outputs": {
    "enabled": true,
    "session_name": "simflow_case"
  }
})json";
}

} // namespace

int main(int argc, char* argv[]) {
    (void)argc;
    try {
        test_support::registerBuiltinComponentTypes();
        auto& simflow_registry =
            gnc::simflow::SimFlowMaterializerRegistry::instance();
        simflow_registry.clearForTests();
        gnc::bootstrap::registerBuiltinSimFlowMaterializers(simflow_registry);

        const fs::path root = "user/outputs/test_simflow_runner";
        std::error_code ec;
        fs::remove_all(root, ec);
        writeFile(root / "base_mission.json", baseMissionJson());
        writeFile(root / "cases.csv",
                  "case_id,engine.temp_level,aero.drag_bias\n"
                  "hot,2,-0.03\n"
                  "cold,0,0.02\n");
        writeFile(root / "simflow.json", R"json({
  "base_mission": "user/outputs/test_simflow_runner/base_mission.json",
  "materializer": {
    "type": "simflow.materializer.numeric_perturbation",
    "config": {
      "case_source": {
        "mode": "matrix",
        "file": "user/outputs/test_simflow_runner/cases.csv",
        "rows": [0, 1]
      },
      "vehicles": {
        "vehicle": {
          "inputs": ["engine.temp_level", "aero.drag_bias"]
        }
      }
    }
  },
  "outputs": {
    "directory": "user/outputs/test_simflow_runner/run",
    "case_directory": "case_{case_index}"
  }
})json");

        gnc::simflow::SimFlowRunner runner;
        runner.runSerial((root / "simflow.json").string());

        test_support::require(
            fs::exists(root / "run" / "case_000001" / "effective_mission.json"),
            "SimFlow should write case 1 effective mission.");
        test_support::require(
            !fs::exists(root / "run" / "case_000001" / "case_manifest.json"),
            "SimFlow should not write case manifests; effective_mission.json is the replay contract.");
        test_support::require(
            fs::exists(root / "run" / "case_000001" / "simflow_case.csv"),
            "SimFlow case 1 should write normal simulation outputs.");
        const auto summary = readText(root / "run" / "simflow_summary.csv");
        test_support::require(summary.find("case_000001") != std::string::npos &&
                                  summary.find("case_000002") != std::string::npos,
                              "SimFlow summary should include both cases.");
        const auto replay_mission =
            fs::absolute(root / "run" / "case_000001" / "effective_mission.json");
        const auto replay_command =
            gncSimPath(argv[0]).generic_string() + " --config " +
            replay_mission.generic_string();
        test_support::require(std::system(replay_command.c_str()) == 0,
                              "Effective mission should replay through plain --config.");

        runner.runMultiprocess((root / "simflow.json").string(),
                               gncSimPath(argv[0]).string(),
                               2);
        const auto parallel_summary =
            readText(root / "run" / "simflow_summary.csv");
        test_support::require(parallel_summary.find("succeeded") != std::string::npos,
                              "Multiprocess simflow should record succeeded cases.");

        const auto repo_root =
            gnc::simflow::findRepoRoot(fs::absolute(root / "simflow.json"));
        const auto repo_output =
            repo_root / "user" / "outputs" / "test_simflow_runner" / "repo_run";
        fs::remove_all(repo_output, ec);
        writeFile(root / "repo_output_simflow.json", R"json({
  "base_mission": "user/outputs/test_simflow_runner/base_mission.json",
  "materializer": {
    "type": "simflow.materializer.numeric_perturbation",
    "config": {
      "case_source": {
        "mode": "matrix",
        "file": "user/outputs/test_simflow_runner/cases.csv",
        "rows": [0]
      },
      "vehicles": {
        "vehicle": {
          "inputs": ["engine.temp_level", "aero.drag_bias"]
        }
      }
    }
  },
  "outputs": {
    "directory": "repo://user/outputs/test_simflow_runner/repo_run",
    "case_directory": "case_{case_index}"
  }
})json");
        runner.runSerial((root / "repo_output_simflow.json").string());
        test_support::require(
            fs::exists(repo_output / "case_000001" / "effective_mission.json"),
            "SimFlow outputs.directory should resolve repo:// roots before writing cases.");
        test_support::require(
            !fs::exists(repo_output / "case_000001" / "case_manifest.json"),
            "SimFlow repo:// output should not write case manifests.");

        std::cout << "simflow runner checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
