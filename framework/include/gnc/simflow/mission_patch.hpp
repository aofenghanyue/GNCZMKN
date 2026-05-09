#pragma once

#include "gnc/core/config_manager.hpp"

#include <stdexcept>
#include <string>
#include <unordered_map>

namespace gnc::simflow {

inline gnc::core::ConfigNode cloneNode(const gnc::core::ConfigNode& node) {
    if (node.isNull()) {
        return gnc::core::ConfigNode();
    }
    if (node.isBool()) {
        return gnc::core::ConfigNode::makeBool(node.asBool());
    }
    if (node.isNumber()) {
        return gnc::core::ConfigNode::makeNumber(node.asDouble());
    }
    if (node.isString()) {
        return gnc::core::ConfigNode::makeString(node.asString());
    }
    if (node.isArray()) {
        auto array = gnc::core::ConfigNode::makeArray();
        for (size_t i = 0; i < node.size(); ++i) {
            array.push(cloneNode(node[i]));
        }
        return array;
    }
    auto object = gnc::core::ConfigNode::makeObject();
    for (const auto& [key, value] : node) {
        object.set(key, cloneNode(value));
    }
    return object;
}

inline gnc::core::ConfigNode numericInputsObject(
    const std::unordered_map<std::string, double>& inputs) {
    auto object = gnc::core::ConfigNode::makeObject();
    for (const auto& [key, value] : inputs) {
        object.set(key, gnc::core::ConfigNode::makeNumber(value));
    }
    return object;
}

inline gnc::core::ConfigNode injectNumericPerturbationInputs(
    const gnc::core::ConfigNode& root,
    const std::string& vehicle_id,
    const std::unordered_map<std::string, double>& inputs) {
    auto out = gnc::core::ConfigNode::makeObject();
    for (const auto& [key, value] : root) {
        if (key != "vehicles") {
            out.set(key, cloneNode(value));
            continue;
        }
        if (!value.isArray()) {
            throw std::runtime_error("simflow mission patch requires root.vehicles to be an array.");
        }

        auto vehicles = gnc::core::ConfigNode::makeArray();
        bool found_vehicle = false;
        for (size_t i = 0; i < value.size(); ++i) {
            const auto& vehicle = value[i];
            auto vehicle_copy = cloneNode(vehicle);
            if (vehicle["id"].asString() == vehicle_id) {
                found_vehicle = true;
                const auto& perturbation = vehicle["perturbation"];
                if (perturbation.isNull()) {
                    throw std::runtime_error("simflow vehicle '" + vehicle_id +
                                             "' requires base mission perturbation block.");
                }
                auto perturbation_copy = cloneNode(perturbation);
                auto config_copy = cloneNode(perturbation["config"]);
                if (config_copy.isNull()) {
                    config_copy = gnc::core::ConfigNode::makeObject();
                }
                config_copy.set("inputs", numericInputsObject(inputs));
                perturbation_copy.set("config", config_copy);
                vehicle_copy.set("perturbation", perturbation_copy);
            }
            vehicles.push(vehicle_copy);
        }
        if (!found_vehicle) {
            throw std::runtime_error("simflow references unknown vehicle '" +
                                     vehicle_id + "'.");
        }
        out.set("vehicles", vehicles);
    }
    return out;
}

inline gnc::core::ConfigNode rewriteOutputDirectory(
    const gnc::core::ConfigNode& root,
    const std::string& output_directory) {
    auto out = cloneNode(root);
    auto outputs = cloneNode(out["outputs"]);
    if (outputs.isNull()) {
        outputs = gnc::core::ConfigNode::makeObject();
    }
    if (!outputs.isObject()) {
        throw std::runtime_error("simflow mission patch requires root.outputs to be an object.");
    }
    outputs.set("directory", gnc::core::ConfigNode::makeString(output_directory));
    out.set("outputs", outputs);
    return out;
}

} // namespace gnc::simflow
