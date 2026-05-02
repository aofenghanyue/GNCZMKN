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

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace gnc::runset {

class RunSetRunner {
public:
    void runSerial(const std::string& runset_file) const {
        gnc::core::ConfigManager runset_manager;
        if (!runset_manager.loadFromFile(runset_file)) {
            throw std::runtime_error("Failed to load runset file '" + runset_file + "'.");
        }
        const auto config = parseRunSetConfig(runset_manager.root());

        gnc::core::ConfigManager base_manager;
        if (!base_manager.loadFromFile(config.base_mission)) {
            throw std::runtime_error("Failed to load base mission '" +
                                     config.base_mission + "'.");
        }

        std::unordered_map<std::string, std::vector<MatrixCase>> vehicle_cases;
        size_t case_count = 0;
        bool first_vehicle = true;
        for (const auto& [vehicle_id, source] : config.vehicles) {
            if (source.mode != "matrix") {
                throw std::runtime_error(
                    "Only matrix runset mode is supported in phase 1.");
            }
            const auto text = readFile(source.file);
            auto cases = parseMatrixCases(text, source.rows);
            if (first_vehicle) {
                case_count = cases.size();
                first_vehicle = false;
            } else if (cases.size() != case_count) {
                throw std::runtime_error(
                    "All vehicle case sources must produce the same case count.");
            }
            vehicle_cases[vehicle_id] = std::move(cases);
        }

        std::filesystem::create_directories(config.output_directory);
        for (size_t i = 0; i < case_count; ++i) {
            auto effective = base_manager.root();
            std::unordered_map<std::string, gnc::core::ConfigNode> inputs_by_vehicle;
            for (const auto& [vehicle_id, cases] : vehicle_cases) {
                effective = overlayVehiclePerturbationInputs(effective,
                                                             vehicle_id,
                                                             cases[i].inputs);
                inputs_by_vehicle[vehicle_id] = numericInputsObject(cases[i].inputs);
            }

            const auto case_dir = std::filesystem::path(config.output_directory) /
                                  formatCaseDirectory(config.case_directory, i + 1);
            std::filesystem::create_directories(case_dir);
            effective = overlayCaseOutputDirectory(effective, case_dir);
            const auto effective_json = writeJson(effective);
            writeFile(case_dir / "effective_mission.json", effective_json);
            writeFile(case_dir / "perturbation_inputs.json",
                      writeVehicleInputsJson(inputs_by_vehicle));

            gnc::core::SimulationBuilder builder;
            if (!builder.loadConfigString(effective_json)) {
                throw std::runtime_error("Generated effective mission did not parse.");
            }
            auto& simulator = builder.build();
            simulator.run();
            writeResolvedSnapshots(case_dir, simulator.getRegistry());
        }
    }

private:
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
        const std::unordered_map<std::string, gnc::core::ConfigNode>& inputs_by_vehicle) {
        auto root = gnc::core::ConfigNode::makeObject();
        for (const auto& [vehicle_id, inputs] : inputs_by_vehicle) {
            root.set(vehicle_id, inputs);
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

    static void writeResolvedSnapshots(const std::filesystem::path& case_dir,
                                       const gnc::core::ComponentRegistry& registry) {
        auto root = gnc::core::ConfigNode::makeObject();
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
        }
        writeFile(case_dir / "perturbation_resolved.json", writeJson(root));
    }
};

} // namespace gnc::runset
