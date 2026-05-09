#pragma once

#include "gnc/simflow/case_directory.hpp"
#include "gnc/simflow/csv_case_source.hpp"
#include "gnc/simflow/mission_patch.hpp"
#include "gnc/simflow/path_utils.hpp"
#include "gnc/simflow/simflow_materializer.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace gnc::simflow {

class NumericPerturbationMaterializer final : public ISimFlowMaterializer {
public:
    std::vector<MaterializedCase> materialize(
        const SimFlowMaterializationContext& context) override {
        const auto cases = loadCases(context);
        const auto vehicle_inputs = parseVehicleInputs(context.materializer_config);

        std::vector<MaterializedCase> out;
        out.reserve(cases.size());
        for (size_t i = 0; i < cases.size(); ++i) {
            const auto& variables = cases[i];
            auto mission = context.base_mission;
            for (const auto& [vehicle_id, input_names] : vehicle_inputs) {
                std::unordered_map<std::string, double> inputs;
                for (const auto& input_name : input_names) {
                    const auto value_it = variables.values.find(input_name);
                    if (value_it == variables.values.end()) {
                        throw std::runtime_error(
                            "simflow numeric materializer case '" +
                            variables.case_id + "' is missing input '" +
                            input_name + "'.");
                    }
                    if (value_it->second.type != SimFlowCaseValue::Type::Number) {
                        throw std::runtime_error(
                            "simflow numeric materializer input '" +
                            input_name + "' must be numeric.");
                    }
                    inputs[input_name] = value_it->second.number;
                }
                mission = injectNumericPerturbationInputs(mission, vehicle_id, inputs);
            }

            const auto case_dir_name =
                formatCaseDirectory(context.case_directory_pattern,
                                    i + 1,
                                    variables.case_id);
            mission = rewriteOutputDirectory(
                mission,
                (context.output_directory / case_dir_name).generic_string());

            MaterializedCase current;
            current.case_id = variables.case_id;
            current.effective_mission = mission;
            out.push_back(std::move(current));
        }
        return out;
    }

private:
    using VehicleInputs = std::unordered_map<std::string, std::vector<std::string>>;

    static std::string readFile(const std::filesystem::path& path) {
        std::ifstream file(path, std::ios::binary);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to read simflow case source: " +
                                     path.generic_string());
        }
        std::ostringstream buffer;
        buffer << file.rdbuf();
        return buffer.str();
    }

    static size_t parseIndex(const gnc::core::ConfigNode& node,
                             const std::string& context,
                             bool allow_zero) {
        const double value = node.asDouble();
        if (!node.isNumber() || !std::isfinite(value) || value < 0.0 ||
            std::floor(value) != value ||
            (!allow_zero && value == 0.0)) {
            throw std::runtime_error(context + " must be a " +
                                     (allow_zero ? "non-negative" : "positive") +
                                     " integer.");
        }
        return static_cast<size_t>(value);
    }

    static std::vector<size_t> parseRows(const gnc::core::ConfigNode& rows) {
        if (!rows.isArray()) {
            throw std::runtime_error(
                "simflow numeric materializer case_source.rows must be an array.");
        }
        std::vector<size_t> out;
        for (size_t i = 0; i < rows.size(); ++i) {
            out.push_back(parseIndex(rows[i],
                                     "simflow numeric materializer case_source.rows[" +
                                         std::to_string(i) + "]",
                                     true));
        }
        if (out.empty()) {
            throw std::runtime_error(
                "simflow numeric materializer case_source.rows must not be empty.");
        }
        return out;
    }

    static SimFlowCaseValue numberValue(double value) {
        SimFlowCaseValue out;
        out.type = SimFlowCaseValue::Type::Number;
        out.number = value;
        return out;
    }

    static double finiteNumber(const gnc::core::ConfigNode& node,
                               const std::string& context) {
        if (!node.isNumber() || !std::isfinite(node.asDouble())) {
            throw std::runtime_error(context + " must be a finite number.");
        }
        return node.asDouble();
    }

    static long long integerNumber(const gnc::core::ConfigNode& node,
                                   const std::string& context) {
        const double value = finiteNumber(node, context);
        if (std::floor(value) != value) {
            throw std::runtime_error(context + " must be an integer.");
        }
        return static_cast<long long>(value);
    }

    static double randomValue(const gnc::core::ConfigNode& spec,
                              const std::string& context,
                              std::mt19937_64& rng) {
        if (spec.isNumber()) {
            return finiteNumber(spec, context);
        }
        if (!spec.isObject()) {
            throw std::runtime_error(context +
                                     " must be a number or distribution object.");
        }
        const std::string distribution = spec["distribution"].asString();
        if (distribution == "constant") {
            return finiteNumber(spec["value"], context + ".value");
        }
        if (distribution == "uniform") {
            const double min = finiteNumber(spec["min"], context + ".min");
            const double max = finiteNumber(spec["max"], context + ".max");
            if (max < min) {
                throw std::runtime_error(context + ".max must be >= min.");
            }
            std::uniform_real_distribution<double> dist(min, max);
            return dist(rng);
        }
        if (distribution == "normal") {
            const double mean = finiteNumber(spec["mean"], context + ".mean");
            const double stddev = finiteNumber(spec["stddev"], context + ".stddev");
            if (stddev < 0.0) {
                throw std::runtime_error(context + ".stddev must be >= 0.");
            }
            std::normal_distribution<double> dist(mean, stddev);
            return dist(rng);
        }
        if (distribution == "uniform_int") {
            const auto min = integerNumber(spec["min"], context + ".min");
            const auto max = integerNumber(spec["max"], context + ".max");
            if (max < min) {
                throw std::runtime_error(context + ".max must be >= min.");
            }
            std::uniform_int_distribution<long long> dist(min, max);
            return static_cast<double>(dist(rng));
        }
        throw std::runtime_error(context +
                                 ".distribution must be constant, uniform, normal, or uniform_int.");
    }

    static std::vector<SimFlowCaseVariables> singleCases(
        const gnc::core::ConfigNode& case_source) {
        const auto& inputs = case_source["inputs"];
        if (!inputs.isObject() || inputs.size() == 0) {
            throw std::runtime_error(
                "simflow numeric materializer single inputs must be a non-empty object.");
        }
        SimFlowCaseVariables current;
        current.case_id = case_source["case_id"].asString("case_000001");
        for (const auto& [key, value] : inputs) {
            current.values[key] = numberValue(finiteNumber(value, "single.inputs." + key));
        }
        return {current};
    }

    static std::vector<SimFlowCaseVariables> randomCases(
        const gnc::core::ConfigNode& case_source) {
        const size_t count =
            parseIndex(case_source["count"], "random.count", false);
        const auto seed =
            static_cast<unsigned long long>(parseIndex(case_source["seed"],
                                                       "random.seed",
                                                       true));
        const auto& inputs = case_source["inputs"];
        if (!inputs.isObject() || inputs.size() == 0) {
            throw std::runtime_error(
                "simflow numeric materializer random inputs must be a non-empty object.");
        }
        std::vector<std::string> keys;
        for (const auto& [key, value] : inputs) {
            (void)value;
            keys.push_back(key);
        }
        std::sort(keys.begin(), keys.end());

        std::mt19937_64 rng(seed);
        std::vector<SimFlowCaseVariables> out;
        for (size_t i = 0; i < count; ++i) {
            SimFlowCaseVariables current;
            current.case_id = "case_" + zeroPaddedCaseIndex(i + 1);
            for (const auto& key : keys) {
                current.values[key] = numberValue(
                    randomValue(inputs[key], "random.inputs." + key, rng));
            }
            out.push_back(std::move(current));
        }
        return out;
    }

    static std::vector<SimFlowCaseVariables> loadCases(
        const SimFlowMaterializationContext& context) {
        const auto& case_source = context.materializer_config["case_source"];
        if (!case_source.isObject()) {
            throw std::runtime_error(
                "simflow numeric materializer requires case_source object.");
        }
        const std::string mode = case_source["mode"].asString();
        if (mode == "single") {
            return singleCases(case_source);
        }
        if (mode == "random") {
            return randomCases(case_source);
        }
        if (mode == "matrix") {
            const std::string file = case_source["file"].asString();
            if (file.empty()) {
                throw std::runtime_error(
                    "simflow numeric materializer matrix case_source.file is required.");
            }
            const auto path = resolvePath(file,
                                          context.simflow_file.empty()
                                              ? std::filesystem::current_path() / "simflow.json"
                                              : context.simflow_file,
                                          context.repo_root.empty()
                                              ? std::filesystem::current_path()
                                              : context.repo_root,
                                          context.project_root);
            return parseCsvCaseSource(readFile(path), parseRows(case_source["rows"]));
        }
        throw std::runtime_error("Unsupported simflow numeric materializer mode '" +
                                 mode + "'.");
    }

    static VehicleInputs parseVehicleInputs(const gnc::core::ConfigNode& config) {
        const auto& vehicles = config["vehicles"];
        if (!vehicles.isObject() || vehicles.size() == 0) {
            throw std::runtime_error(
                "simflow numeric materializer vehicles must be a non-empty object.");
        }
        VehicleInputs out;
        for (const auto& [vehicle_id, vehicle_config] : vehicles) {
            const auto& inputs = vehicle_config["inputs"];
            if (!inputs.isArray() || inputs.size() == 0) {
                throw std::runtime_error(
                    "simflow numeric materializer vehicles." + vehicle_id +
                    ".inputs must be a non-empty string array.");
            }
            for (size_t i = 0; i < inputs.size(); ++i) {
                const std::string input = inputs[i].asString();
                if (input.empty()) {
                    throw std::runtime_error(
                        "simflow numeric materializer vehicles." + vehicle_id +
                        ".inputs[" + std::to_string(i) +
                        "] must be a non-empty string.");
                }
                out[vehicle_id].push_back(input);
            }
        }
        return out;
    }

};

} // namespace gnc::simflow
