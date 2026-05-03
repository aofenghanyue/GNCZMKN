#include "test_support.hpp"

#include "gnc/runset/runset_runner.hpp"

#include <exception>
#include <cstdlib>
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

int runCommand(const std::string& command) {
    return std::system(command.c_str());
}

fs::path gncSimPath(const char* argv0) {
    fs::path executable = fs::absolute(fs::path(argv0)).parent_path() / "gnc_sim";
#ifdef _WIN32
    executable += ".exe";
#endif
    return executable;
}

std::string baseMissionJson(bool outputs_enabled = false) {
    return std::string(R"json(
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
              "1": "nominal.txt",
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
  "outputs": { "enabled": )json") +
           (outputs_enabled ? "true" : "false") + R"json( }
}
)json";
}

} // namespace

int main(int argc, char* argv[]) {
    (void)argc;
    try {
        test_support::registerBuiltinComponentTypes();

        const fs::path root = fs::path("user/outputs/test_runset_full");
        std::error_code ec;
        fs::remove_all(root, ec);
        writeFile(root / "base_mission.json", baseMissionJson());

        writeFile(root / "single_runset.json", R"json(
{
  "base_mission": "user/outputs/test_runset_full/base_mission.json",
  "vehicles": {
    "vehicle": {
      "cases": {
        "mode": "single",
        "inputs": {
          "engine.temp_level": 2,
          "aero.drag_bias": -0.03
        }
      }
    }
  },
  "outputs": {
    "directory": "user/outputs/test_runset_full/single_run",
    "case_directory": "case_{case_index}"
  }
}
)json");

        gnc::runset::RunSetRunner runner;
        runner.runSerial((root / "single_runset.json").string());

        const auto single_effective =
            readText(root / "single_run" / "case_000001" / "effective_mission.json");
        test_support::require(single_effective.find("\"aero.drag_bias\"") !=
                                  std::string::npos,
                              "Single source did not inject numeric inputs.");
        test_support::require(
            !fs::exists(root / "single_run" / "case_000001" / "generated_case_row.csv"),
            "RunSet should not duplicate generated_cases.csv into each case directory.");
        test_support::require(
            !fs::exists(root / "single_run" / "runset_manifest.json"),
            "RunSet should not write a manifest when generated case files are sufficient.");
        const auto single_summary =
            readText(root / "single_run" / "runset_summary.csv");
        test_support::require(single_summary.find("succeeded") != std::string::npos,
                              "Single source summary did not record success.");

        auto failure_base = baseMissionJson();
        const std::string hot_map_entry = ",\n              \"2\": \"hot.txt\"";
        const auto hot_map_pos = failure_base.find(hot_map_entry);
        test_support::require(hot_map_pos != std::string::npos,
                              "Failed to prepare failure-continuation fixture.");
        failure_base.replace(hot_map_pos, hot_map_entry.size(), "");
        writeFile(root / "failure_base.json", failure_base);
        writeFile(root / "failure_matrix.csv",
                  "case_id,engine.temp_level,aero.drag_bias\n"
                  "invalid,2,-0.03\n"
                  "valid,0,0.02\n");
        writeFile(root / "failure_runset.json", R"json(
{
  "base_mission": "user/outputs/test_runset_full/failure_base.json",
  "vehicles": {
    "vehicle": {
      "cases": {
        "mode": "matrix",
        "file": "user/outputs/test_runset_full/failure_matrix.csv",
        "rows": [0, 1]
      }
    }
  },
  "outputs": {
    "directory": "user/outputs/test_runset_full/failure_run",
    "case_directory": "case_{case_index}"
  }
}
)json");
        bool failure_run_threw = false;
        try {
            runner.runSerial((root / "failure_runset.json").string());
        } catch (const std::exception&) {
            failure_run_threw = true;
        }
        test_support::require(failure_run_threw,
                              "RunSet should return failure when any case fails.");
        const auto failure_summary =
            readText(root / "failure_run" / "runset_summary.csv");
        test_support::require(failure_summary.find("failed") != std::string::npos &&
                                  failure_summary.find("succeeded") != std::string::npos,
                              "RunSet summary should record failed and continued cases.");
        test_support::require(
            fs::exists(root / "failure_run" / "case_000002" / "effective_mission.json"),
            "RunSet should continue after a failed case and execute later cases.");

        const std::string random_template = R"json(
{
  "base_mission": "user/outputs/test_runset_full/base_mission.json",
  "vehicles": {
    "vehicle": {
      "cases": {
        "mode": "random",
        "seed": 12345,
        "count": 3,
        "inputs": {
          "engine.temp_level": {
            "distribution": "uniform_int",
            "min": 0,
            "max": 2
          },
          "aero.drag_bias": {
            "distribution": "uniform",
            "min": -0.05,
            "max": 0.05
          }
        }
      }
    }
  },
  "outputs": {
    "directory": "OUTPUT_DIR",
    "case_directory": "case_{case_index}"
  }
}
)json";
        auto random_a = random_template;
        random_a.replace(random_a.find("OUTPUT_DIR"), std::string("OUTPUT_DIR").size(),
                         "user/outputs/test_runset_full/random_a");
        auto random_b = random_template;
        random_b.replace(random_b.find("OUTPUT_DIR"), std::string("OUTPUT_DIR").size(),
                         "user/outputs/test_runset_full/random_b");
        writeFile(root / "random_a.json", random_a);
        writeFile(root / "random_b.json", random_b);

        runner.runSerial((root / "random_a.json").string());
        runner.runSerial((root / "random_b.json").string());
        test_support::require(
            readText(root / "random_a" / "generated_cases.csv") ==
                readText(root / "random_b" / "generated_cases.csv"),
            "Random source with the same seed should reproduce generated cases.");
        test_support::require(
            !fs::exists(root / "random_a" / "runset_manifest.json"),
            "Random source should rely on generated_cases.csv and effective missions, not a manifest.");

        writeFile(root / "matrix.csv",
                  "case_id,engine.temp_level,aero.drag_bias\n"
                  "hot,2,-0.03\n"
                  "cold,0,0.02\n");
        writeFile(root / "jobs_runset.json", R"json(
{
  "base_mission": "user/outputs/test_runset_full/base_mission.json",
  "vehicles": {
    "vehicle": {
      "cases": {
        "mode": "matrix",
        "file": "user/outputs/test_runset_full/matrix.csv",
        "rows": [0, 1]
      }
    }
  },
  "outputs": {
    "directory": "user/outputs/test_runset_full/jobs_run",
    "case_directory": "case_{case_index}"
  }
}
)json");

        runner.runMultiprocess((root / "jobs_runset.json").string(),
                               gncSimPath(argv[0]).string(),
                               2);
        const auto jobs_summary = readText(root / "jobs_run" / "runset_summary.csv");
        test_support::require(jobs_summary.find("case_000001") != std::string::npos &&
                                  jobs_summary.find("case_000002") != std::string::npos,
                              "Multiprocess summary should include both cases.");
        test_support::require(
            !fs::exists(root / "jobs_run" / "case_000001" / "perturbation_resolved.json"),
            "Multiprocess child should not write RunSet snapshot artifacts through --config.");
        test_support::require(
            !fs::exists(root / "jobs_run" / "case_000002" / "perturbation_resolved.json"),
            "Multiprocess child should not write RunSet snapshot artifacts through --config.");

        const auto replay_mission =
            fs::absolute(root / "jobs_run" / "case_000001" / "effective_mission.json");
        const auto replay_command =
            gncSimPath(argv[0]).generic_string() + " --config " +
            replay_mission.generic_string();
        test_support::require(runCommand(replay_command) == 0,
                              "A generated effective mission should replay through plain --config.");
        test_support::require(
            !fs::exists(root / "jobs_run" / "case_000001" / "perturbation_resolved.json"),
            "Plain --config replay should not create RunSet snapshot artifacts.");

        std::cout << "full runset checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
