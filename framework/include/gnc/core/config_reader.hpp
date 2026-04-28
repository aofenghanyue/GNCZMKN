#pragma once

#include "gnc/core/config_manager.hpp"

#include <initializer_list>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace gnc::core {

class ConfigReader {
public:
    ConfigReader(const ConfigNode& node, std::string path)
        : node_(node), path_(std::move(path)) {}

    bool has(const std::string& key) const {
        return node_.isObject() && node_.has(key);
    }

    double requiredDouble(const std::string& key) const {
        const auto& child = node_[key];
        if (!child.isNumber()) {
            throw std::runtime_error(fieldPath(key) +
                                     " is required and must be a number.");
        }
        return child.asDouble();
    }

    double optionalDouble(const std::string& key, double default_value) const {
        const auto& child = node_[key];
        if (child.isNull()) {
            return default_value;
        }
        if (!child.isNumber()) {
            throw std::runtime_error(fieldPath(key) + " must be a number.");
        }
        return child.asDouble();
    }

    int optionalInt(const std::string& key, int default_value) const {
        const auto& child = node_[key];
        if (child.isNull()) {
            return default_value;
        }
        if (!child.isNumber()) {
            throw std::runtime_error(fieldPath(key) + " must be a number.");
        }
        return child.asInt();
    }

    bool optionalBool(const std::string& key, bool default_value) const {
        const auto& child = node_[key];
        if (child.isNull()) {
            return default_value;
        }
        if (!child.isBool()) {
            throw std::runtime_error(fieldPath(key) + " must be a boolean.");
        }
        return child.asBool();
    }

    std::string requiredString(const std::string& key) const {
        const auto& child = node_[key];
        if (!child.isString() || child.asString().empty()) {
            throw std::runtime_error(fieldPath(key) +
                                     " is required and must be a non-empty string.");
        }
        return child.asString();
    }

    std::string optionalString(const std::string& key,
                               const std::string& default_value) const {
        const auto& child = node_[key];
        if (child.isNull()) {
            return default_value;
        }
        if (!child.isString()) {
            throw std::runtime_error(fieldPath(key) + " must be a string.");
        }
        return child.asString();
    }

    ConfigReader requiredObject(const std::string& key) const {
        const auto& child = node_[key];
        if (!child.isObject()) {
            throw std::runtime_error(fieldPath(key) +
                                     " is required and must be an object.");
        }
        child.resetAccessTracking();
        return ConfigReader(child, fieldPath(key));
    }

    std::vector<double> requiredDoubleArray(const std::string& key,
                                            size_t exact_size = 0) const {
        const auto& child = node_[key];
        if (!child.isArray()) {
            throw std::runtime_error(fieldPath(key) +
                                     " is required and must be an array.");
        }
        return readDoubleArray(child, fieldPath(key), exact_size);
    }

    std::vector<double> optionalDoubleArray(const std::string& key,
                                            std::vector<double> default_value,
                                            size_t exact_size = 0) const {
        const auto& child = node_[key];
        if (child.isNull()) {
            return default_value;
        }
        if (!child.isArray()) {
            throw std::runtime_error(fieldPath(key) + " must be an array.");
        }
        return readDoubleArray(child, fieldPath(key), exact_size);
    }

    void validateNoUnknownKeys() const {
        validateNoUnknownKeys(node_, path_);
    }

    static void validateNoUnknownKeys(const ConfigNode& node,
                                      const std::string& path) {
        const auto unused = node.getUnusedKeys();
        if (unused.empty()) {
            return;
        }

        std::ostringstream oss;
        oss << path << " has unrecognized config key";
        if (unused.size() > 1) {
            oss << "s";
        }
        oss << ": ";
        for (size_t i = 0; i < unused.size(); ++i) {
            if (i > 0) {
                oss << ", ";
            }
            oss << unused[i];
        }
        oss << ".";
        throw std::runtime_error(oss.str());
    }

private:
    std::string fieldPath(const std::string& key) const {
        if (path_.empty()) {
            return key;
        }
        return path_ + "." + key;
    }

    static std::vector<double> readDoubleArray(const ConfigNode& node,
                                               const std::string& path,
                                               size_t exact_size) {
        if (exact_size > 0 && node.size() != exact_size) {
            throw std::runtime_error(path + " must contain exactly " +
                                     std::to_string(exact_size) + " numbers.");
        }

        std::vector<double> values;
        values.reserve(node.size());
        for (size_t i = 0; i < node.size(); ++i) {
            const auto& item = node[i];
            if (!item.isNumber()) {
                throw std::runtime_error(path + "[" + std::to_string(i) +
                                         "] must be a number.");
            }
            values.push_back(item.asDouble());
        }
        return values;
    }

    const ConfigNode& node_;
    std::string path_;
};

} // namespace gnc::core
