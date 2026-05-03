#pragma once

#include "gnc/core/config_manager.hpp"
#include "gnc/runset/matrix_case_source.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace gnc::runset {

struct VehicleCaseSourceConfig {
    std::string mode;
    std::string file;
    std::vector<size_t> rows;
    gnc::core::ConfigNode inputs;
    size_t count = 0;
    unsigned long long seed = 0;
};

struct RunSetConfig {
    std::string base_mission;
    std::string output_directory = "user/outputs";
    std::string case_directory = "case_{case_index}";
    std::unordered_map<std::string, VehicleCaseSourceConfig> vehicles;
};

inline size_t parseSizeIndex(const gnc::core::ConfigNode& node,
                             const std::string& context,
                             bool allow_zero) {
    const double value = node.asDouble();
    if (!node.isNumber() || !std::isfinite(value) || value < 0.0 ||
        std::floor(value) != value ||
        value > static_cast<double>(std::numeric_limits<size_t>::max()) ||
        (!allow_zero && value == 0.0)) {
        throw std::runtime_error(context + " must be a " +
                                 (allow_zero ? "non-negative" : "positive") +
                                 " integer.");
    }
    return static_cast<size_t>(value);
}

inline void validateNumericInputsObject(const gnc::core::ConfigNode& inputs,
                                        const std::string& context) {
    if (!inputs.isObject()) {
        throw std::runtime_error(context + " must be an object.");
    }
    for (const auto& [key, value] : inputs) {
        if (!value.isNumber() || !std::isfinite(value.asDouble())) {
            throw std::runtime_error(context + "." + key +
                                     " must be a finite number.");
        }
    }
}

inline RunSetConfig parseRunSetConfig(const gnc::core::ConfigNode& root) {
    RunSetConfig config;
    config.base_mission = root["base_mission"].asString();
    if (config.base_mission.empty()) {
        throw std::runtime_error("runset.base_mission is required.");
    }

    const auto& outputs = root["outputs"];
    if (!outputs.isNull()) {
        config.output_directory =
            outputs["directory"].asString(config.output_directory);
        config.case_directory =
            outputs["case_directory"].asString(config.case_directory);
    }

    const auto& vehicles = root["vehicles"];
    if (!vehicles.isObject()) {
        throw std::runtime_error("runset.vehicles must be an object.");
    }

    for (const auto& [vehicle_id, vehicle_node] : vehicles) {
        const auto& cases = vehicle_node["cases"];
        VehicleCaseSourceConfig source;
        source.mode = cases["mode"].asString();
        if (source.mode.empty()) {
            throw std::runtime_error("runset.vehicles." + vehicle_id +
                                     ".cases.mode is required.");
        }

        if (source.mode == "single") {
            source.inputs = cases["inputs"];
            validateNumericInputsObject(
                source.inputs,
                "runset.vehicles." + vehicle_id + ".cases.inputs");
        } else if (source.mode == "matrix") {
            source.file = cases["file"].asString();
            if (source.file.empty()) {
                throw std::runtime_error("runset.vehicles." + vehicle_id +
                                         ".cases.file is required for matrix mode.");
            }
            const auto& rows = cases["rows"];
            if (!rows.isArray()) {
                throw std::runtime_error("runset.vehicles." + vehicle_id +
                                         ".cases.rows must be an array.");
            }
            for (size_t i = 0; i < rows.size(); ++i) {
                source.rows.push_back(parseSizeIndex(
                    rows[i],
                    "runset.vehicles." + vehicle_id + ".cases.rows[" +
                        std::to_string(i) + "]",
                    true));
            }
        } else if (source.mode == "random") {
            source.inputs = cases["inputs"];
            if (!source.inputs.isObject()) {
                throw std::runtime_error("runset.vehicles." + vehicle_id +
                                         ".cases.inputs must be an object.");
            }
            source.count = parseSizeIndex(
                cases["count"],
                "runset.vehicles." + vehicle_id + ".cases.count",
                false);
            source.seed = static_cast<unsigned long long>(parseSizeIndex(
                cases["seed"],
                "runset.vehicles." + vehicle_id + ".cases.seed",
                true));
        } else {
            throw std::runtime_error("Unsupported runset mode '" + source.mode +
                                     "' for vehicle '" + vehicle_id + "'.");
        }

        if (source.mode == "matrix" && source.rows.empty()) {
            throw std::runtime_error("runset.vehicles." + vehicle_id +
                                     ".cases.rows must contain at least one row.");
        }
        if (source.mode == "single" && source.inputs.size() == 0) {
            throw std::runtime_error("runset.vehicles." + vehicle_id +
                                     ".cases.inputs must contain at least one input.");
        }
        if (source.mode == "random" && source.inputs.size() == 0) {
            throw std::runtime_error("runset.vehicles." + vehicle_id +
                                     ".cases.inputs must contain at least one input.");
        }

        config.vehicles[vehicle_id] = std::move(source);
    }

    if (config.vehicles.empty()) {
        throw std::runtime_error("runset.vehicles must contain at least one vehicle.");
    }

    return config;
}

inline std::unordered_map<std::string, double> numericInputsFromConfig(
    const gnc::core::ConfigNode& inputs,
    const std::string& context) {
    validateNumericInputsObject(inputs, context);
    std::unordered_map<std::string, double> result;
    for (const auto& [key, value] : inputs) {
        result[key] = value.asDouble();
    }
    return result;
}

inline std::vector<std::string> sortedObjectKeys(const gnc::core::ConfigNode& object) {
    std::vector<std::string> keys;
    if (!object.isObject()) {
        return keys;
    }
    for (const auto& [key, value] : object) {
        (void)value;
        keys.push_back(key);
    }
    std::sort(keys.begin(), keys.end());
    return keys;
}

inline double requiredFiniteDouble(const gnc::core::ConfigNode& node,
                                   const std::string& context) {
    if (!node.isNumber() || !std::isfinite(node.asDouble())) {
        throw std::runtime_error(context + " must be a finite number.");
    }
    return node.asDouble();
}

inline long long requiredInteger(const gnc::core::ConfigNode& node,
                                 const std::string& context) {
    const double value = requiredFiniteDouble(node, context);
    if (std::floor(value) != value ||
        value < static_cast<double>(std::numeric_limits<long long>::min()) ||
        value > static_cast<double>(std::numeric_limits<long long>::max())) {
        throw std::runtime_error(context + " must be an integer.");
    }
    return static_cast<long long>(value);
}

inline double generateRandomValue(const gnc::core::ConfigNode& spec,
                                  const std::string& context,
                                  std::mt19937_64& rng) {
    if (spec.isNumber()) {
        return requiredFiniteDouble(spec, context);
    }
    if (!spec.isObject()) {
        throw std::runtime_error(context +
                                 " must be a number or random distribution object.");
    }

    const std::string distribution = spec["distribution"].asString();
    if (distribution == "constant") {
        return requiredFiniteDouble(spec["value"], context + ".value");
    }
    if (distribution == "uniform") {
        const double min = requiredFiniteDouble(spec["min"], context + ".min");
        const double max = requiredFiniteDouble(spec["max"], context + ".max");
        if (max < min) {
            throw std::runtime_error(context + ".max must be >= min.");
        }
        std::uniform_real_distribution<double> dist(min, max);
        return dist(rng);
    }
    if (distribution == "normal") {
        const double mean = requiredFiniteDouble(spec["mean"], context + ".mean");
        const double stddev = requiredFiniteDouble(spec["stddev"], context + ".stddev");
        if (stddev < 0.0) {
            throw std::runtime_error(context + ".stddev must be >= 0.");
        }
        std::normal_distribution<double> dist(mean, stddev);
        return dist(rng);
    }
    if (distribution == "uniform_int") {
        const auto min = requiredInteger(spec["min"], context + ".min");
        const auto max = requiredInteger(spec["max"], context + ".max");
        if (max < min) {
            throw std::runtime_error(context + ".max must be >= min.");
        }
        std::uniform_int_distribution<long long> dist(min, max);
        return static_cast<double>(dist(rng));
    }

    throw std::runtime_error(context +
                             ".distribution must be constant, uniform, normal, or uniform_int.");
}

inline std::vector<MatrixCase> generateRandomCases(
    const VehicleCaseSourceConfig& source,
    const std::string& vehicle_id) {
    std::mt19937_64 rng(source.seed);
    const auto keys = sortedObjectKeys(source.inputs);
    std::vector<MatrixCase> result;
    result.reserve(source.count);

    for (size_t i = 0; i < source.count; ++i) {
        MatrixCase current;
        for (const auto& key : keys) {
            current.inputs[key] = generateRandomValue(
                source.inputs[key],
                "runset.vehicles." + vehicle_id + ".cases.inputs." + key,
                rng);
        }
        result.push_back(std::move(current));
    }
    return result;
}

inline std::vector<MatrixCase> singleCaseSource(
    const VehicleCaseSourceConfig& source,
    const std::string& vehicle_id) {
    MatrixCase current;
    current.inputs = numericInputsFromConfig(
        source.inputs,
        "runset.vehicles." + vehicle_id + ".cases.inputs");
    return {std::move(current)};
}

inline std::vector<MatrixCase> loadCasesForSource(
    const VehicleCaseSourceConfig& source,
    const std::string& vehicle_id,
    const std::function<std::string(const std::string&)>& read_file) {
    if (source.mode == "single") {
        return singleCaseSource(source, vehicle_id);
    }
    if (source.mode == "matrix") {
        return parseMatrixCases(read_file(source.file), source.rows);
    }
    if (source.mode == "random") {
        return generateRandomCases(source, vehicle_id);
    }
    throw std::runtime_error("Unsupported runset mode '" + source.mode +
                             "' for vehicle '" + vehicle_id + "'.");
}

} // namespace gnc::runset
