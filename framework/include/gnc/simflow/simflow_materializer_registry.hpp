#pragma once

#include "gnc/simflow/simflow_materializer.hpp"

#include <algorithm>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace gnc::simflow {

class SimFlowMaterializerRegistry {
public:
    using Factory = std::function<std::unique_ptr<ISimFlowMaterializer>()>;

    static SimFlowMaterializerRegistry& instance() {
        static SimFlowMaterializerRegistry registry;
        return registry;
    }

    template <typename T>
    void registerType(const std::string& type_id) {
        registerFactory(type_id, [] { return std::make_unique<T>(); });
    }

    void registerFactory(const std::string& type_id, Factory factory) {
        if (type_id.empty()) {
            throw std::runtime_error("SimFlow materializer type id must not be empty.");
        }
        if (!factory) {
            throw std::runtime_error("SimFlow materializer factory must not be empty.");
        }
        if (factories_.count(type_id) > 0) {
            throw std::runtime_error("SimFlow materializer type already registered: " +
                                     type_id);
        }
        factories_[type_id] = std::move(factory);
    }

    bool hasType(const std::string& type_id) const {
        return factories_.count(type_id) > 0;
    }

    std::unique_ptr<ISimFlowMaterializer> create(const std::string& type_id) const {
        const auto it = factories_.find(type_id);
        if (it == factories_.end()) {
            throw std::runtime_error("Unknown SimFlow materializer type: " + type_id);
        }
        return it->second();
    }

    std::vector<std::string> listTypes() const {
        std::vector<std::string> out;
        for (const auto& [type_id, factory] : factories_) {
            (void)factory;
            out.push_back(type_id);
        }
        std::sort(out.begin(), out.end());
        return out;
    }

    void clearForTests() {
        factories_.clear();
    }

private:
    std::unordered_map<std::string, Factory> factories_;
};

} // namespace gnc::simflow
