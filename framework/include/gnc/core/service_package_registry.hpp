#pragma once

#include "gnc/core/component_registry.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/core/service_context.hpp"

#include <algorithm>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace gnc::core {

enum class ServiceScopeKind {
    Global,
    Environment,
    Vehicle
};

inline const char* toString(ServiceScopeKind kind) {
    switch (kind) {
    case ServiceScopeKind::Global:
        return "global";
    case ServiceScopeKind::Environment:
        return "environment";
    case ServiceScopeKind::Vehicle:
        return "vehicle";
    default:
        return "unknown";
    }
}

struct ServiceScopeInfo {
    ServiceScopeKind kind = ServiceScopeKind::Global;
    std::string name;
    std::string registry_scope;
    std::string config_path;
};

using ServiceDiagnosticReporter = std::function<void(const std::string&)>;

struct ServiceCreationContext {
    ServiceContext& services;
    const ConfigNode& config;
    ServiceScopeInfo scope;
};

struct ServiceFinalizationContext {
    ComponentRegistry& registry;
    ServiceDiagnosticReporter add_warning;
};

class IServiceFinalizationTask {
public:
    virtual ~IServiceFinalizationTask() = default;
    virtual void finalize(ServiceFinalizationContext& context) = 0;
};

class IServicePackage {
public:
    virtual ~IServicePackage() = default;

    virtual std::string_view id() const = 0;
    virtual std::vector<ServiceScopeKind> supportedScopes() const = 0;
    virtual std::unique_ptr<IServiceFinalizationTask> create(
        const ServiceCreationContext& context) const = 0;

    bool supportsScope(ServiceScopeKind kind) const {
        const auto scopes = supportedScopes();
        return std::find(scopes.begin(), scopes.end(), kind) != scopes.end();
    }
};

class ServicePackageRegistry {
public:
    void registerPackage(std::unique_ptr<IServicePackage> package) {
        if (!package) {
            throw std::runtime_error("Cannot register a null service package.");
        }

        const std::string service_id(package->id());
        if (service_id.empty()) {
            throw std::runtime_error("Service package id cannot be empty.");
        }
        if (packages_.count(service_id) > 0) {
            throw std::runtime_error("Service package '" + service_id +
                                     "' is already registered.");
        }

        packages_.emplace(service_id, std::move(package));
    }

    const IServicePackage* findPackage(std::string_view id) const {
        const auto it = packages_.find(std::string(id));
        if (it == packages_.end()) {
            return nullptr;
        }
        return it->second.get();
    }

    std::vector<std::string> listPackageIds() const {
        std::vector<std::string> ids;
        ids.reserve(packages_.size());
        for (const auto& [id, _] : packages_) {
            ids.push_back(id);
        }
        std::sort(ids.begin(), ids.end());
        return ids;
    }

private:
    std::unordered_map<std::string, std::unique_ptr<IServicePackage>> packages_;
};

inline std::string joinServiceScopeNames(
    const std::vector<ServiceScopeKind>& scopes) {
    std::string result;
    for (size_t i = 0; i < scopes.size(); ++i) {
        if (i > 0) {
            result += ", ";
        }
        result += "'";
        result += toString(scopes[i]);
        result += "'";
    }
    return result.empty() ? "(none)" : result;
}

} // namespace gnc::core
