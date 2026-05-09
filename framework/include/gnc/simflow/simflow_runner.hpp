#pragma once

#include "gnc/core/config_manager.hpp"
#include "gnc/core/simulation_builder.hpp"
#include "gnc/simflow/case_directory.hpp"
#include "gnc/simflow/json_writer.hpp"
#include "gnc/simflow/mission_patch.hpp"
#include "gnc/simflow/path_utils.hpp"
#include "gnc/simflow/simflow_config.hpp"
#include "gnc/simflow/simflow_materializer_registry.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace gnc::simflow {

class SimFlowRunner {
public:
    void runSerial(const std::string& simflow_file) const {
        auto plan = prepareSimFlow(simflow_file);
        std::vector<CaseExecutionResult> results;
        results.reserve(plan.cases.size());

        for (const auto& current_case : plan.cases) {
            results.push_back(runCaseInProcess(current_case));
        }

        writeSimFlowSummary(plan.output_directory, results);
        throwIfAnyFailed(results);
    }

    void runMultiprocess(const std::string& simflow_file,
                         const std::string& executable_path,
                         size_t jobs) const {
        if (jobs == 0) {
            throw std::runtime_error("SimFlow multiprocess jobs must be >= 1.");
        }

        auto plan = prepareSimFlow(simflow_file);
        std::vector<CaseExecutionResult> results(plan.cases.size());
        std::vector<RunningChild> running;
        size_t next_case = 0;

        while (next_case < plan.cases.size() || !running.empty()) {
            while (next_case < plan.cases.size() && running.size() < jobs) {
                const auto& current_case = plan.cases[next_case];
                const auto command = childCommand(executable_path,
                                                  current_case.effective_mission_path);
                running.push_back(
                    RunningChild{next_case,
                                 std::async(std::launch::async,
                                            [command]() {
                                                return std::system(command.c_str());
                                            })});
                ++next_case;
            }

            bool collected = false;
            for (auto it = running.begin(); it != running.end();) {
                if (it->future.wait_for(std::chrono::milliseconds(0)) !=
                    std::future_status::ready) {
                    ++it;
                    continue;
                }

                const auto plan_index = it->plan_index;
                const int exit_code = normalizeSystemExitCode(it->future.get());
                results[plan_index] =
                    resultFromExitCode(plan.cases[plan_index], exit_code);
                it = running.erase(it);
                collected = true;
            }

            if (!collected && !running.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }

        writeSimFlowSummary(plan.output_directory, results);
        throwIfAnyFailed(results);
    }

private:
    struct PreparedCase {
        size_t case_index = 0;
        std::string case_id;
        std::filesystem::path case_dir;
        std::filesystem::path effective_mission_path;
    };

    struct PreparedSimFlow {
        std::filesystem::path output_directory;
        std::vector<PreparedCase> cases;
    };

    struct CaseExecutionResult {
        size_t case_index = 0;
        std::string case_id;
        std::filesystem::path case_dir;
        std::string status = "failed";
        int exit_code = 1;
        std::string message;
    };

    struct RunningChild {
        size_t plan_index = 0;
        std::future<int> future;
    };

    PreparedSimFlow prepareSimFlow(const std::string& simflow_file) const {
        gnc::core::ConfigManager simflow_manager;
        if (!simflow_manager.loadFromFile(simflow_file)) {
            throw std::runtime_error("Failed to load simflow file '" +
                                     simflow_file + "'.");
        }
        auto config = parseSimFlowConfig(simflow_manager.root());

        const auto simflow_path = absolutePath(std::filesystem::path(simflow_file));
        const auto repo_root = findRepoRoot(simflow_path);
        const auto project_root = findProjectRoot(simflow_path, repo_root);
        const auto base_mission_path =
            resolvePath(config.base_mission, simflow_path, repo_root, project_root);

        gnc::core::ConfigManager base_manager;
        if (!base_manager.loadFromFile(base_mission_path.string())) {
            throw std::runtime_error("Failed to load simflow base mission '" +
                                     base_mission_path.generic_string() + "'.");
        }

        auto materializer =
            SimFlowMaterializerRegistry::instance().create(config.materializer_type);
        SimFlowMaterializationContext context;
        context.simflow_config = simflow_manager.root();
        context.materializer_config = config.materializer_config;
        context.base_mission = base_manager.root();
        context.simflow_file = simflow_path;
        context.repo_root = repo_root;
        context.project_root = project_root;
        context.output_directory = resolveOutputDirectory(config.output_directory,
                                                          simflow_path,
                                                          repo_root,
                                                          project_root);
        context.case_directory_pattern = config.case_directory;

        auto materialized_cases = materializer->materialize(context);
        if (materialized_cases.empty()) {
            throw std::runtime_error("SimFlow materializer produced no cases.");
        }

        PreparedSimFlow plan;
        plan.output_directory = context.output_directory;
        std::filesystem::create_directories(plan.output_directory);
        plan.cases.reserve(materialized_cases.size());

        for (size_t i = 0; i < materialized_cases.size(); ++i) {
            plan.cases.push_back(prepareCase(materialized_cases[i],
                                             config,
                                             plan.output_directory,
                                             i + 1));
        }

        return plan;
    }

    static PreparedCase prepareCase(const MaterializedCase& materialized,
                                    const SimFlowConfig& config,
                                    const std::filesystem::path& output_directory,
                                    size_t case_index) {
        const std::string case_id =
            materialized.case_id.empty()
                ? "case_" + zeroPaddedCaseIndex(case_index)
                : materialized.case_id;
        const std::string case_dir_name =
            formatCaseDirectory(config.case_directory, case_index, case_id);

        PreparedCase prepared;
        prepared.case_index = case_index;
        prepared.case_id = case_id;
        prepared.case_dir = output_directory / case_dir_name;
        std::filesystem::create_directories(prepared.case_dir);

        const auto effective_mission =
            rewriteOutputDirectory(materialized.effective_mission,
                                   prepared.case_dir.generic_string());
        prepared.effective_mission_path =
            prepared.case_dir / "effective_mission.json";
        writeFile(prepared.effective_mission_path, writeJson(effective_mission));
        return prepared;
    }

    static CaseExecutionResult runCaseInProcess(const PreparedCase& current_case) {
        CaseExecutionResult result;
        result.case_index = current_case.case_index;
        result.case_id = current_case.case_id;
        result.case_dir = current_case.case_dir;

        try {
            gnc::core::SimulationBuilder builder;
            if (!builder.loadConfig(current_case.effective_mission_path.string())) {
                throw std::runtime_error("Generated effective mission did not parse.");
            }
            auto& simulator = builder.build();
            simulator.run();
            result.status = "succeeded";
            result.exit_code = 0;
            return result;
        } catch (const std::exception& ex) {
            result.status = "failed";
            result.exit_code = 1;
            result.message = ex.what();
            return result;
        }
    }

    static CaseExecutionResult resultFromExitCode(const PreparedCase& current_case,
                                                  int exit_code) {
        CaseExecutionResult result;
        result.case_index = current_case.case_index;
        result.case_id = current_case.case_id;
        result.case_dir = current_case.case_dir;
        result.exit_code = exit_code;
        result.status = exit_code == 0 ? "succeeded" : "failed";
        if (exit_code != 0) {
            result.message = "child process exited with code " +
                             std::to_string(exit_code);
        }
        return result;
    }

    static std::filesystem::path absolutePath(const std::filesystem::path& path) {
        std::error_code ec;
        auto out = std::filesystem::absolute(path, ec);
        if (ec) {
            out = path;
        }
        return out.lexically_normal();
    }

    static std::filesystem::path resolveOutputDirectory(
        const std::string& value,
        const std::filesystem::path& simflow_path,
        const std::filesystem::path& repo_root,
        const std::filesystem::path& project_root) {
        if (startsWith(value, "repo://") ||
            startsWith(value, "project://") ||
            startsWith(value, "user-data://")) {
            return resolvePath(value, simflow_path, repo_root, project_root);
        }

        const std::filesystem::path requested(value);
        if (requested.is_absolute()) {
            return absolutePath(requested);
        }
        return requested.lexically_normal();
    }

    static void writeFile(const std::filesystem::path& path,
                          const std::string& text) {
        std::ofstream file(path, std::ios::binary);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to write file '" +
                                     path.generic_string() + "'.");
        }
        file << text;
    }

    static std::string escapeCsv(std::string value) {
        const bool needs_quotes =
            value.find_first_of(",\"\n\r") != std::string::npos;
        if (!needs_quotes) {
            return value;
        }
        std::string escaped = "\"";
        for (char ch : value) {
            if (ch == '"') {
                escaped += "\"\"";
            } else {
                escaped += ch;
            }
        }
        escaped += "\"";
        return escaped;
    }

    static void writeSimFlowSummary(
        const std::filesystem::path& output_directory,
        const std::vector<CaseExecutionResult>& results) {
        std::ostringstream out;
        out << "case_index,case_id,case_directory,status,exit_code,message\n";
        for (const auto& result : results) {
            out << result.case_index << ","
                << escapeCsv(result.case_id) << ","
                << escapeCsv(result.case_dir.generic_string()) << ","
                << result.status << ","
                << result.exit_code << ","
                << escapeCsv(result.message) << "\n";
        }
        writeFile(output_directory / "simflow_summary.csv", out.str());
    }

    static void throwIfAnyFailed(
        const std::vector<CaseExecutionResult>& results) {
        const auto failed =
            std::find_if(results.begin(),
                         results.end(),
                         [](const CaseExecutionResult& result) {
                             return result.exit_code != 0;
                         });
        if (failed != results.end()) {
            throw std::runtime_error(
                "One or more SimFlow cases failed; see simflow_summary.csv.");
        }
    }

    static std::string quoteCommandArgument(const std::string& value) {
        std::string result = "\"";
        for (char ch : value) {
            if (ch == '"') {
                result += "\\\"";
            } else {
                result += ch;
            }
        }
        result += "\"";
        return result;
    }

    static std::string childCommand(const std::string& executable_path,
                                    const std::filesystem::path& mission_path) {
        const auto command =
            quoteCommandArgument(executable_path) + " --config " +
            quoteCommandArgument(mission_path.generic_string());
#ifdef _WIN32
        return "\"" + command + "\"";
#else
        return command;
#endif
    }

    static int normalizeSystemExitCode(int raw_code) {
        return raw_code == 0 ? 0 : raw_code;
    }
};

} // namespace gnc::simflow
