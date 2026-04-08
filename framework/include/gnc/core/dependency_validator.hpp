/**
 * @file dependency_validator.hpp
 * @brief 依赖验证器
 * 
 * 在仿真初始化阶段检查组件依赖关系是否满足
 */
#pragma once

#include "component_registry.hpp"
#include "string_utils.hpp"
#include "gnc/common/logger.hpp"
#include <algorithm>
#include <vector>
#include <string>
#include <typeindex>
#include <unordered_set>

namespace gnc::core {

/**
 * @brief 依赖声明
 * 
 * 组件可以声明它需要的依赖接口
 */
struct DependencyDeclaration {
    std::type_index interface_type;
    std::string description;
    bool required;
    std::string lookup_name;
};

template<typename Interface>
DependencyDeclaration requireDependency(const std::string& lookup_name,
                                        const std::string& description = "") {
    return {
        std::type_index(typeid(Interface)),
        description,
        true,
        lookup_name
    };
}

template<typename Interface>
DependencyDeclaration optionalDependency(const std::string& lookup_name,
                                         const std::string& description = "") {
    return {
        std::type_index(typeid(Interface)),
        description,
        false,
        lookup_name
    };
}

/**
 * @brief 可声明依赖的组件接口
 * 
 * 组件可以选择性地实现此接口来声明依赖
 */
/**
 * @brief Optional dependency-contract layer for richer diagnostics
 *
 * Most components do not need to implement this interface. Required
 * `bind(...)` and optional `bindIfPresent(...)` bindings are already
 * preflight-checked during `SimulationBuilder::build()`.
 *
 * Implement this interface when the extra declaration adds real value:
 * - the component has several dependencies and you want semantic,
 *   human-readable descriptions in diagnostics
 * - you need interface-existence checks that are not tied to a specific
 *   lookup name
 * - you want an explicit dependency contract for tooling or documentation
 *
 * Declarations do not replace real bindings. Any dependency listed in
 * `getDependencies()` should still have a matching binding in
 * `injectDependencies()`, and the framework validates both layers.
 */
class IDependencyDeclarer {
public:
    virtual ~IDependencyDeclarer() = default;
    virtual std::vector<DependencyDeclaration> getDependencies() const = 0;
};

/**
 * @brief 依赖验证器
 * 
 * 检查所有组件的依赖是否满足
 */
class DependencyValidator {
public:
    struct ValidationResult {
        bool success = true;
        std::vector<std::string> errors;
        std::vector<std::string> warnings;
        std::unordered_set<std::string> failed_components;
    };
    
    /**
     * @brief 验证所有组件的依赖
     */
    static ValidationResult validate(const ComponentRegistry& registry) {
        ValidationResult result;
        
        for (auto* component : registry.getAllComponents()) {
            auto* declarer = dynamic_cast<IDependencyDeclarer*>(component);
            if (!declarer) continue;
            
            auto deps = declarer->getDependencies();
            for (const auto& dep : deps) {
                bool found = dep.lookup_name.empty()
                             ? checkInterfaceExists(registry, dep.interface_type)
                             : checkDependencyLookup(registry, component->getName(), dep, result);
                
                if (!found && dep.lookup_name.empty()) {
                    const std::string description =
                        dep.description.empty() ? dep.interface_type.name() : dep.description;
                    std::string msg = "Component '" + component->getName() +
                                     "' requires " + description +
                                     " (interface: " + std::string(dep.interface_type.name()) + ")" +
                                     " but no registered component provides this interface.\n"
                                     "  Registered components: " + joinNames(registry) + "\n"
                                     "  Suggestion: Add a component implementing this interface to your configuration.";
                    if (dep.required) {
                        result.errors.push_back(msg);
                        result.success = false;
                        result.failed_components.insert(component->getName());
                    } else {
                        result.warnings.push_back(msg + "\n  Optional dependency: the component can still run without it.");
                    }
                }
            }
        }
        
        if (result.success && result.warnings.empty()) {
            LOG_INFO("Dependency validation passed");
        }
        
        return result;
    }
    
private:
    static std::string joinStrings(const std::vector<std::string>& values) {
        std::string result;
        for (size_t i = 0; i < values.size(); ++i) {
            if (i > 0) result += ", ";
            result += values[i];
        }
        return result.empty() ? "(none)" : result;
    }

    static std::string joinNames(const ComponentRegistry& registry) {
        return joinStrings(registry.getComponentNames());
    }

    static std::string extractScope(const std::string& component_name) {
        const auto pos = component_name.rfind('.');
        if (pos == std::string::npos) {
            return "";
        }
        return component_name.substr(0, pos);
    }

    static std::string resolveLookupName(const std::string& owner_name,
                                         const std::string& lookup_name) {
        if (lookup_name.find('.') != std::string::npos) {
            return lookup_name;
        }

        const std::string scope = extractScope(owner_name);
        if (scope.empty()) {
            return lookup_name;
        }
        return scope + "." + lookup_name;
    }

    static std::vector<std::string> getScopedComponentNames(const ComponentRegistry& registry,
                                                            const std::string& owner_name) {
        std::vector<std::string> result;
        const std::string scope = extractScope(owner_name);
        for (const auto& name : registry.getComponentNames()) {
            if (extractScope(name) == scope) {
                result.push_back(name);
            }
        }
        return result;
    }

    static std::vector<std::string> getScopedProviders(const ComponentRegistry& registry,
                                                       const std::string& owner_name,
                                                       std::type_index type_idx) {
        std::vector<std::string> result;
        const std::string scope = extractScope(owner_name);
        for (const auto& name : registry.getComponentsForInterface(type_idx)) {
            if (extractScope(name) == scope) {
                result.push_back(name);
            }
        }
        return result;
    }

    static bool checkDependencyLookup(const ComponentRegistry& registry,
                                      const std::string& owner_name,
                                      const DependencyDeclaration& dep,
                                      ValidationResult& result) {
        const std::string resolved_name = resolveLookupName(owner_name, dep.lookup_name);
        const bool component_exists = registry.has(resolved_name);
        const auto providers = registry.getComponentsForInterface(dep.interface_type);
        const bool interface_found = std::find(providers.begin(), providers.end(), resolved_name) != providers.end();

        if (interface_found) {
            return true;
        }

        const std::string description =
            dep.description.empty() ? dep.interface_type.name() : dep.description;
        std::string msg = "Component '" + owner_name + "' requires " + description +
                         " via lookup '" + dep.lookup_name + "' (resolved as '" + resolved_name + "')";

        if (component_exists) {
            msg += ", but component '" + resolved_name + "' does not implement interface '" +
                   std::string(dep.interface_type.name()) + "'.";
        } else {
            msg += ", but no component with that name is registered.";
        }

        const auto scope_components = getScopedComponentNames(registry, owner_name);
        msg += "\n  Same-scope components: " + joinStrings(scope_components);

        const auto scoped_providers = getScopedProviders(registry, owner_name, dep.interface_type);
        msg += "\n  Same-scope providers for this interface: " + joinStrings(scoped_providers);
        msg += "\n  Registered providers for this interface: " + joinStrings(providers);

        const std::string suggestion = findClosestMatch(resolved_name, registry.getComponentNames());
        if (!suggestion.empty()) {
            msg += "\n  Did you mean '" + suggestion + "'?";
        }

        if (dep.required) {
            result.errors.push_back(msg);
            result.success = false;
            result.failed_components.insert(owner_name);
        } else {
            result.warnings.push_back(msg + "\n  Optional dependency: the component can still run without it.");
        }
        return false;
    }

    static bool checkInterfaceExists(const ComponentRegistry& registry,
                                     std::type_index type_idx) {
        return registry.hasInterface(type_idx);
    }
};

/**
 * @brief 执行阶段管理器
 * 
 * 确保组件按正确的阶段执行
 */
class ExecutionPhaseManager {
public:
    enum class Phase {
        NotStarted,
        Initializing,
        Running,
        Finalizing,
        Completed
    };
    
    Phase getCurrentPhase() const { return current_phase_; }
    
    void transitionTo(Phase phase) {
        // 验证状态转换是否合法
        if (!isValidTransition(current_phase_, phase)) {
            LOG_ERROR("Invalid phase transition: {} -> {}", 
                      phaseToString(current_phase_), phaseToString(phase));
            return;
        }
        
        LOG_INFO("Simulation phase: {} -> {}", 
                 phaseToString(current_phase_), phaseToString(phase));
        current_phase_ = phase;
    }
    
private:
    Phase current_phase_ = Phase::NotStarted;
    
    static bool isValidTransition(Phase from, Phase to) {
        switch (from) {
            case Phase::NotStarted:
                return to == Phase::Initializing;
            case Phase::Initializing:
                return to == Phase::Running;
            case Phase::Running:
                return to == Phase::Finalizing;
            case Phase::Finalizing:
                return to == Phase::Completed;
            case Phase::Completed:
                return to == Phase::NotStarted; // 重置
            default:
                return false;
        }
    }
    
    static const char* phaseToString(Phase phase) {
        switch (phase) {
            case Phase::NotStarted: return "NotStarted";
            case Phase::Initializing: return "Initializing";
            case Phase::Running: return "Running";
            case Phase::Finalizing: return "Finalizing";
            case Phase::Completed: return "Completed";
            default: return "Unknown";
        }
    }
};

} // namespace gnc::core
