#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/perturbation/interfaces/i_perturbation_provider.hpp"

#include <cmath>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace gnc::perturbation {

class StaticPerturbation final : public gnc::core::ComponentBase,
                                 public IPerturbationProvider,
                                 public IPerturbationSnapshot {
public:
    StaticPerturbation() : ComponentBase("StaticPerturbation") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        const auto& inputs = config["inputs"];
        if (!inputs.isNull() && !inputs.isObject()) {
            throw std::runtime_error(config_path + ".inputs must be an object.");
        }

        for (const auto& [key, value] : inputs) {
            if (!value.isNumber()) {
                throw std::runtime_error(config_path + ".inputs." + key +
                                         " must be a number.");
            }
            PerturbationValue output;
            output.type = PerturbationValue::Type::Number;
            output.number = value.asDouble();
            values_[key] = output;
        }

        const auto& enum_maps = config["enum_maps"];
        if (!enum_maps.isNull() && !enum_maps.isObject()) {
            throw std::runtime_error(config_path + ".enum_maps must be an object.");
        }

        for (const auto& [key, map_node] : enum_maps) {
            if (!map_node.isObject()) {
                throw std::runtime_error(config_path + ".enum_maps." + key +
                                         " must be an object.");
            }
            const auto input_it = values_.find(key);
            if (input_it == values_.end()) {
                continue;
            }
            const auto index_text = integerKey(input_it->second.number, config_path, key);
            const auto& string_node = map_node[index_text];
            if (!string_node.isString()) {
                throw std::runtime_error(config_path + ".enum_maps." + key +
                                         " does not define key '" + index_text + "'.");
            }
            PerturbationValue output;
            output.type = PerturbationValue::Type::String;
            output.string = string_node.asString();
            values_[key + ".resolved"] = output;
        }

        gnc::core::ConfigReader(config, config_path).validateNoUnknownKeys();
    }

    void update(double) override {}

    bool has(const std::string& key) const override {
        return values_.count(key) > 0;
    }

    double getNumber(const std::string& key, double fallback) const override {
        const auto it = values_.find(key);
        if (it == values_.end() || it->second.type != PerturbationValue::Type::Number) {
            return fallback;
        }
        return it->second.number;
    }

    std::string getString(const std::string& key,
                          const std::string& fallback) const override {
        const auto it = values_.find(key);
        if (it == values_.end() || it->second.type != PerturbationValue::Type::String) {
            return fallback;
        }
        return it->second.string;
    }

    std::vector<double> getVector(const std::string& key) const override {
        const auto it = values_.find(key);
        if (it == values_.end() || it->second.type != PerturbationValue::Type::Vector) {
            return {};
        }
        return it->second.vector;
    }

    std::map<std::string, PerturbationValue> snapshotResolvedState() const override {
        return values_;
    }

private:
    static std::string integerKey(double value,
                                  const std::string& config_path,
                                  const std::string& key) {
        if (!std::isfinite(value) || std::floor(value) != value) {
            throw std::runtime_error(config_path + ".inputs." + key +
                                     " must be an integer to index enum_maps.");
        }
        return std::to_string(static_cast<int>(value));
    }

    std::map<std::string, PerturbationValue> values_;
};

} // namespace gnc::perturbation
