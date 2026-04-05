/**
 * @file dependency_validator.hpp
 * @brief 依赖验证器
 * 
 * 在仿真初始化阶段检查组件依赖关系是否满足
 */
#pragma once

#include "component_registry.hpp"
#include "gnc/common/logger.hpp"
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
};

/**
 * @brief 可声明依赖的组件接口
 * 
 * 组件可以选择性地实现此接口来声明依赖
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
                bool found = checkInterfaceExists(registry, dep.interface_type);
                
                if (!found) {
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
    static std::string joinNames(const ComponentRegistry& registry) {
        const auto& names = registry.getComponentNames();
        std::string result;
        for (size_t i = 0; i < names.size(); ++i) {
            if (i > 0) result += ", ";
            result += names[i];
        }
        return result.empty() ? "(none)" : result;
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
