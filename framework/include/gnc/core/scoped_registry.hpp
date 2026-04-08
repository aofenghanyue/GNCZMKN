#pragma once

#include "component_registry.hpp"
#include "string_utils.hpp"
#include "gnc/common/logger.hpp"
#include <stdexcept>
#include <string>
#include <typeindex>
#include <utility>
#include <vector>

namespace gnc::core {

class ScopedRegistry;

template<typename Interface>
struct DependencyBinding {
    Interface*& slot;
    std::string lookup_name;

    void apply(const ScopedRegistry& registry) const;
};

template<typename Interface>
struct OptionalDependencyBinding {
    Interface*& slot;
    std::string lookup_name;

    void apply(const ScopedRegistry& registry) const;
};

template<typename Interface>
DependencyBinding<Interface> bind(Interface*& slot, std::string lookup_name) {
    return {slot, std::move(lookup_name)};
}

template<typename Interface>
OptionalDependencyBinding<Interface> bindIfPresent(Interface*& slot, std::string lookup_name) {
    return {slot, std::move(lookup_name)};
}

class ScopedRegistry {
public:
    struct BindingDiagnostics {
        std::vector<std::string> errors;
        std::vector<std::string> warnings;
    };

    ScopedRegistry(const std::string& scope,
                   ComponentRegistry& registry,
                   std::string requester_name = {},
                   BindingDiagnostics* diagnostics = nullptr)
        : scope_(normalizeScope(scope)),
          registry_(registry),
          requester_name_(std::move(requester_name)),
          diagnostics_(diagnostics) {}

    template<typename Interface>
    Interface* getByName(const std::string& name) const {
        auto* result = tryGetByName<Interface>(name);
        if (!result) {
            LOG_WARNING("Component '{}' not found in scope '{}'", name, scope_);
        }
        return result;
    }

    template<typename Interface>
    Interface* tryGetByName(const std::string& name) const {
        return registry_.get<Interface>(resolveName(name));
    }

    template<typename Interface>
    Interface* requireByName(const std::string& name) const {
        auto* result = tryGetByName<Interface>(name);
        if (!result) {
            const std::string message = buildLookupFailureMessage<Interface>(name);
            if (diagnostics_) {
                diagnostics_->errors.push_back(message);
                return nullptr;
            }
            throw std::runtime_error(message);
        }
        return result;
    }

    template<typename Interface>
    std::vector<Interface*> getAll() const {
        std::vector<Interface*> result;

        for (auto* component : registry_.getAllComponents()) {
            if (!component || !isInScope(component->getName())) {
                continue;
            }

            auto* ptr = dynamic_cast<Interface*>(component);
            if (ptr) {
                result.push_back(ptr);
            }
        }

        return result;
    }

    template<typename... Bindings>
    void bindAll(const Bindings&... bindings) const {
        (bindings.apply(*this), ...);
    }

    const std::string& getScope() const { return scope_; }

    ComponentRegistry& getRegistry() { return registry_; }
    const ComponentRegistry& getRegistry() const { return registry_; }

private:
    static std::string joinStrings(const std::vector<std::string>& values) {
        std::string result;
        for (size_t i = 0; i < values.size(); ++i) {
            if (i > 0) result += ", ";
            result += values[i];
        }
        return result.empty() ? "(none)" : result;
    }

    static std::string normalizeScope(std::string scope) {
        if (!scope.empty() && scope.back() != '.') {
            scope.push_back('.');
        }
        return scope;
    }

    std::string resolveName(const std::string& name) const {
        if (name.find('.') != std::string::npos) {
            return name;
        }
        return scope_ + name;
    }

    bool isInScope(const std::string& name) const {
        if (scope_.empty()) {
            return name.find('.') == std::string::npos;
        }
        return name.size() > scope_.size() &&
               name.compare(0, scope_.size(), scope_) == 0;
    }

    std::vector<std::string> getScopedComponentNames() const {
        std::vector<std::string> result;
        for (const auto& name : registry_.getComponentNames()) {
            if (isInScope(name)) {
                result.push_back(name);
            }
        }
        return result;
    }

    template<typename Interface>
    std::vector<std::string> getScopedProviders() const {
        std::vector<std::string> result;
        const auto providers = registry_.getComponentsForInterface(std::type_index(typeid(Interface)));
        for (const auto& name : providers) {
            if (isInScope(name) && name != requester_name_) {
                result.push_back(name);
            }
        }
        return result;
    }

    template<typename Interface>
    std::string buildLookupFailureMessage(const std::string& name) const {
        const std::string resolved_name = resolveName(name);
        std::string message =
            "Required dependency lookup '" + name + "' resolved as '" + resolved_name +
            "' failed for interface '" + std::string(typeid(Interface).name()) + "'.";

        if (registry_.has(resolved_name)) {
            message += " A component with that name exists, but it does not implement the requested interface.";
        } else {
            message += " No component with that name is registered.";
        }

        message += " Same-scope components: " + joinStrings(getScopedComponentNames()) + ".";
        message += " Same-scope providers for this interface: " + joinStrings(getScopedProviders<Interface>()) + ".";

        const std::string suggestion = findClosestMatch(resolved_name, registry_.getComponentNames());
        if (!suggestion.empty()) {
            message += " Did you mean '" + suggestion + "'?";
        }

        return message;
    }

    std::string scope_;
    ComponentRegistry& registry_;
    std::string requester_name_;
    BindingDiagnostics* diagnostics_ = nullptr;
};

template<typename Interface>
inline void DependencyBinding<Interface>::apply(const ScopedRegistry& registry) const {
    slot = registry.requireByName<Interface>(lookup_name);
}

template<typename Interface>
inline void OptionalDependencyBinding<Interface>::apply(const ScopedRegistry& registry) const {
    slot = registry.tryGetByName<Interface>(lookup_name);
}

} // namespace gnc::core
