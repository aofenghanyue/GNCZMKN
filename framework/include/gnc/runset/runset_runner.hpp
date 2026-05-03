#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_registry.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/core/simulation_builder.hpp"
#include "gnc/perturbation/interfaces/i_perturbation_provider.hpp"
#include "gnc/runset/config_json_writer.hpp"
#include "gnc/runset/matrix_case_source.hpp"
#include "gnc/runset/mission_overlay.hpp"
#include "gnc/runset/runset_config.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace gnc::runset {

class RunSetRunner {
public:
    void runSerial(const std::string& runset_file) const {
        auto plan = prepareRunSet(runset_file);
        std::vector<CaseExecutionResult> results;
        results.reserve(plan.cases.size());

        for (const auto& current_case : plan.cases) {
            results.push_back(runCaseInProcess(current_case));
        }

        writeRunSetSummary(plan.output_directory, results);
        throwIfAnyFailed(results);
    }

    void runMultiprocess(const std::string& runset_file,
                         const std::string& executable_path,
                         size_t jobs) const {
        if (jobs == 0) {
            throw std::runtime_error("RunSet multiprocess jobs must be >= 1.");
        }

        auto plan = prepareRunSet(runset_file);
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
                                            [command]() { return std::system(command.c_str()); })});
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

        writeRunSetSummary(plan.output_directory, results);
        throwIfAnyFailed(results);
    }

    static bool writeResolvedSnapshots(const std::filesystem::path& case_dir,
                                       const gnc::core::ComponentRegistry& registry) {
        auto root = gnc::core::ConfigNode::makeObject();
        size_t snapshot_count = 0;
        for (const auto& name : registry.getComponentNames()) {
            auto* component = registry.get<gnc::core::ComponentBase>(name);
            auto* snapshot =
                dynamic_cast<gnc::perturbation::IPerturbationSnapshot*>(component);
            if (!snapshot) {
                continue;
            }

            auto object = gnc::core::ConfigNode::makeObject();
            for (const auto& [key, value] : snapshot->snapshotResolvedState()) {
                object.set(key, perturbationValueToConfigNode(value));
            }
            root.set(name, object);
            ++snapshot_count;
        }

        if (snapshot_count == 0) {
            return false;
        }

        std::filesystem::create_directories(case_dir);
        writeFile(case_dir / "perturbation_resolved.json", writeJson(root));
        return true;
    }

private:
    struct PreparedCase {
        size_t case_index = 0;
        std::filesystem::path case_dir;
        std::filesystem::path effective_mission_path;
        std::unordered_map<std::string, std::unordered_map<std::string, double>>
            inputs_by_vehicle;
    };

    struct PreparedRunSet {
        RunSetConfig config;
        std::filesystem::path output_directory;
        std::vector<PreparedCase> cases;
    };

    struct CaseExecutionResult {
        size_t case_index = 0;
        std::filesystem::path case_dir;
        std::string status = "failed";
        int exit_code = 1;
        std::string message;
    };

    struct RunningChild {
        size_t plan_index = 0;
        std::future<int> future;
    };

    PreparedRunSet prepareRunSet(const std::string& runset_file) const {
        gnc::core::ConfigManager runset_manager;
        if (!runset_manager.loadFromFile(runset_file)) {
            throw std::runtime_error("Failed to load runset file '" + runset_file + "'.");
        }
        auto config = parseRunSetConfig(runset_manager.root());

        gnc::core::ConfigManager base_manager;
        if (!base_manager.loadFromFile(config.base_mission)) {
            throw std::runtime_error("Failed to load base mission '" +
                                     config.base_mission + "'.");
        }

        const auto vehicle_cases = loadVehicleCases(config);
        const auto case_count = validateCaseCount(vehicle_cases);
        PreparedRunSet plan;
        plan.config = config;
        plan.output_directory = std::filesystem::path(config.output_directory);
        std::filesystem::create_directories(plan.output_directory);

        for (size_t i = 0; i < case_count; ++i) {
            plan.cases.push_back(prepareCase(base_manager.root(),
                                             config,
                                             vehicle_cases,
                                             i));
        }

        writeGeneratedCasesCsv(plan.output_directory / "generated_cases.csv", plan.cases);
        writeRunSetManifest(plan.output_directory / "runset_manifest.json",
                            runset_file,
                            config,
                            plan.cases);
        return plan;
    }

    std::unordered_map<std::string, std::vector<MatrixCase>> loadVehicleCases(
        const RunSetConfig& config) const {
        std::unordered_map<std::string, std::vector<MatrixCase>> vehicle_cases;
        for (const auto& [vehicle_id, source] : config.vehicles) {
            vehicle_cases[vehicle_id] = loadCasesForSource(
                source,
                vehicle_id,
                [](const std::string& path) { return readFile(path); });
        }
        return vehicle_cases;
    }

    static size_t validateCaseCount(
        const std::unordered_map<std::string, std::vector<MatrixCase>>& vehicle_cases) {
        size_t case_count = 0;
        bool first_vehicle = true;
        for (const auto& [vehicle_id, cases] : vehicle_cases) {
            if (cases.empty()) {
                throw std::runtime_error("RunSet vehicle '" + vehicle_id +
                                         "' produced no cases.");
            }
            if (first_vehicle) {
                case_count = cases.size();
                first_vehicle = false;
            } else if (cases.size() != case_count) {
                throw std::runtime_error(
                    "All vehicle case sources must produce the same case count.");
            }
        }
        if (first_vehicle) {
            throw std::runtime_error("RunSet must contain at least one vehicle case source.");
        }
        return case_count;
    }

    static PreparedCase prepareCase(
        const gnc::core::ConfigNode& base_root,
        const RunSetConfig& config,
        const std::unordered_map<std::string, std::vector<MatrixCase>>& vehicle_cases,
        size_t case_offset) {
        PreparedCase prepared;
        prepared.case_index = case_offset + 1;
        prepared.case_dir = std::filesystem::path(config.output_directory) /
                            formatCaseDirectory(config.case_directory,
                                                prepared.case_index);
        std::filesystem::create_directories(prepared.case_dir);

        auto effective = base_root;
        for (const auto& [vehicle_id, cases] : vehicle_cases) {
            effective = overlayVehiclePerturbationInputs(effective,
                                                         vehicle_id,
                                                         cases[case_offset].inputs);
            prepared.inputs_by_vehicle[vehicle_id] = cases[case_offset].inputs;
        }
        effective = overlayCaseOutputDirectory(effective, prepared.case_dir);

        const auto effective_json = writeJson(effective);
        prepared.effective_mission_path = prepared.case_dir / "effective_mission.json";
        writeFile(prepared.effective_mission_path, effective_json);
        writeFile(prepared.case_dir / "perturbation_inputs.json",
                  writeVehicleInputsJson(prepared.inputs_by_vehicle));
        writeGeneratedCasesCsv(prepared.case_dir / "generated_case_row.csv",
                               std::vector<PreparedCase>{prepared});
        return prepared;
    }

    static CaseExecutionResult runCaseInProcess(const PreparedCase& current_case) {
        CaseExecutionResult result;
        result.case_index = current_case.case_index;
        result.case_dir = current_case.case_dir;

        try {
            gnc::core::SimulationBuilder builder;
            if (!builder.loadConfig(current_case.effective_mission_path.string())) {
                throw std::runtime_error("Generated effective mission did not parse.");
            }
            auto& simulator = builder.build();
            simulator.run();
            writeResolvedSnapshots(current_case.case_dir, simulator.getRegistry());
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
        result.case_dir = current_case.case_dir;
        result.exit_code = exit_code;
        result.status = exit_code == 0 ? "succeeded" : "failed";
        if (exit_code != 0) {
            result.message = "child process exited with code " + std::to_string(exit_code);
        }
        return result;
    }

    static std::string readFile(const std::string& path) {
        std::ifstream file(path, std::ios::binary);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to read file '" + path + "'.");
        }
        std::ostringstream buffer;
        buffer << file.rdbuf();
        return buffer.str();
    }

    static void writeFile(const std::filesystem::path& path, const std::string& text) {
        std::ofstream file(path, std::ios::binary);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to write file '" +
                                     path.generic_string() + "'.");
        }
        file << text;
    }

    static std::string formatCaseDirectory(std::string pattern, size_t case_index) {
        std::ostringstream index;
        index << std::setw(6) << std::setfill('0') << case_index;
        const std::string token = "{case_index}";
        const auto pos = pattern.find(token);
        if (pos != std::string::npos) {
            pattern.replace(pos, token.size(), index.str());
        }
        return pattern;
    }

    static gnc::core::ConfigNode overlayCaseOutputDirectory(
        const gnc::core::ConfigNode& root,
        const std::filesystem::path& case_dir) {
        if (!root.isObject()) {
            throw std::runtime_error("Generated effective mission root must be an object.");
        }

        auto out = gnc::core::ConfigNode::makeObject();
        bool wrote_outputs = false;
        for (const auto& [key, value] : root) {
            if (key != "outputs") {
                out.set(key, cloneNode(value));
                continue;
            }

            auto outputs = cloneNode(value);
            if (outputs.isNull()) {
                outputs = gnc::core::ConfigNode::makeObject();
            }
            if (!outputs.isObject()) {
                throw std::runtime_error(
                    "Generated effective mission outputs block must be an object.");
            }
            outputs.set("directory",
                        gnc::core::ConfigNode::makeString(case_dir.generic_string()));
            out.set("outputs", outputs);
            wrote_outputs = true;
        }

        if (!wrote_outputs) {
            auto outputs = gnc::core::ConfigNode::makeObject();
            outputs.set("directory",
                        gnc::core::ConfigNode::makeString(case_dir.generic_string()));
            out.set("outputs", outputs);
        }
        return out;
    }

    static std::string writeVehicleInputsJson(
        const std::unordered_map<std::string, std::unordered_map<std::string, double>>&
            inputs_by_vehicle) {
        auto root = gnc::core::ConfigNode::makeObject();
        for (const auto& [vehicle_id, inputs] : inputs_by_vehicle) {
            root.set(vehicle_id, numericInputsObject(inputs));
        }
        return writeJson(root);
    }

    static gnc::core::ConfigNode perturbationValueToConfigNode(
        const gnc::perturbation::PerturbationValue& value) {
        using Type = gnc::perturbation::PerturbationValue::Type;
        if (value.type == Type::Number) {
            return gnc::core::ConfigNode::makeNumber(value.number);
        }
        if (value.type == Type::String) {
            return gnc::core::ConfigNode::makeString(value.string);
        }

        auto array = gnc::core::ConfigNode::makeArray();
        for (const auto entry : value.vector) {
            array.push(gnc::core::ConfigNode::makeNumber(entry));
        }
        return array;
    }

    static std::map<std::string, double> flattenedInputs(const PreparedCase& current_case) {
        std::map<std::string, double> result;
        for (const auto& [vehicle_id, inputs] : current_case.inputs_by_vehicle) {
            for (const auto& [key, value] : inputs) {
                result[vehicle_id + "." + key] = value;
            }
        }
        return result;
    }

    static std::vector<std::string> generatedCaseColumns(
        const std::vector<PreparedCase>& cases) {
        std::set<std::string> columns;
        for (const auto& current_case : cases) {
            for (const auto& [key, value] : flattenedInputs(current_case)) {
                (void)value;
                columns.insert(key);
            }
        }
        return {columns.begin(), columns.end()};
    }

    static void writeGeneratedCasesCsv(const std::filesystem::path& path,
                                       const std::vector<PreparedCase>& cases) {
        const auto columns = generatedCaseColumns(cases);
        std::ostringstream out;
        out << "case_id";
        for (const auto& column : columns) {
            out << "," << column;
        }
        out << "\n";

        for (const auto& current_case : cases) {
            const auto row = flattenedInputs(current_case);
            out << formatCaseDirectory("case_{case_index}", current_case.case_index);
            for (const auto& column : columns) {
                out << ",";
                const auto it = row.find(column);
                if (it != row.end()) {
                    out << std::setprecision(15) << it->second;
                }
            }
            out << "\n";
        }

        writeFile(path, out.str());
    }

    static void writeRunSetManifest(const std::filesystem::path& path,
                                    const std::string& runset_file,
                                    const RunSetConfig& config,
                                    const std::vector<PreparedCase>& cases) {
        auto root = gnc::core::ConfigNode::makeObject();
        root.set("runset_file", gnc::core::ConfigNode::makeString(runset_file));
        root.set("base_mission", gnc::core::ConfigNode::makeString(config.base_mission));
        root.set("output_directory",
                 gnc::core::ConfigNode::makeString(config.output_directory));
        root.set("case_directory",
                 gnc::core::ConfigNode::makeString(config.case_directory));
        root.set("case_count",
                 gnc::core::ConfigNode::makeNumber(static_cast<double>(cases.size())));

        auto case_nodes = gnc::core::ConfigNode::makeArray();
        for (const auto& current_case : cases) {
            auto case_node = gnc::core::ConfigNode::makeObject();
            case_node.set("case_index",
                          gnc::core::ConfigNode::makeNumber(
                              static_cast<double>(current_case.case_index)));
            case_node.set("case_id",
                          gnc::core::ConfigNode::makeString(
                              formatCaseDirectory("case_{case_index}",
                                                  current_case.case_index)));
            case_node.set("case_directory",
                          gnc::core::ConfigNode::makeString(
                              current_case.case_dir.generic_string()));
            case_node.set("effective_mission",
                          gnc::core::ConfigNode::makeString(
                              current_case.effective_mission_path.generic_string()));
            case_nodes.push(case_node);
        }
        root.set("cases", case_nodes);
        writeFile(path, writeJson(root));
    }

    static std::string escapeCsv(std::string value) {
        const bool needs_quotes = value.find_first_of(",\"\n\r") != std::string::npos;
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

    static void writeRunSetSummary(const std::filesystem::path& output_directory,
                                   const std::vector<CaseExecutionResult>& results) {
        std::ostringstream out;
        out << "case_index,case_directory,status,exit_code,message\n";
        for (const auto& result : results) {
            out << result.case_index << ","
                << escapeCsv(result.case_dir.generic_string()) << ","
                << result.status << ","
                << result.exit_code << ","
                << escapeCsv(result.message) << "\n";
        }
        writeFile(output_directory / "runset_summary.csv", out.str());
    }

    static void throwIfAnyFailed(const std::vector<CaseExecutionResult>& results) {
        const auto failed = std::find_if(results.begin(),
                                         results.end(),
                                         [](const CaseExecutionResult& result) {
                                             return result.exit_code != 0;
                                         });
        if (failed != results.end()) {
            throw std::runtime_error(
                "One or more RunSet cases failed; see runset_summary.csv.");
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
        const auto command = quoteCommandArgument(executable_path) + " --config " +
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

} // namespace gnc::runset
