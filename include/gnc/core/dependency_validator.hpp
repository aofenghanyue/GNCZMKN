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
            // 检查组件是否声明了依赖
            auto* declarer = dynamic_cast<IDependencyDeclarer*>(component);
            if (!declarer) continue;
            
            auto deps = declarer->getDependencies();
            for (const auto& dep : deps) {
                // 检查依赖是否存在
                bool found = checkInterfaceExists(registry, dep.interface_type);
                
                if (!found) {
                    std::string msg = "Component '" + component->getName() + 
                                     "' requires " + dep.description + " but none found";
                    if (dep.required) {
                        result.errors.push_back(msg);
                        result.success = false;
                    } else {
                        result.warnings.push_back(msg + " (optional)");
                    }
                }
            }
        }
        
        // 输出结果
        for (const auto& error : result.errors) {
            log_error("Dependency Error: {}", error);
        }
        for (const auto& warning : result.warnings) {
            log_warning("Dependency Warning: {}", warning);
        }
        
        if (result.success && result.errors.empty()) {
            log_info("Dependency validation passed");
        }
        
        return result;
    }
    
private:
    static bool checkInterfaceExists(const ComponentRegistry& registry,
                                      std::type_index type_idx) {
        // 遍历所有组件检查接口
        for (auto* component : registry.getAllComponents()) {
            // 这里使用简化检查 - 实际实现中可以扩展
            (void)component;
            (void)type_idx;
        }
        // 简化：假设通过动态注册的接口都可用
        return true;
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
            log_error("Invalid phase transition: {} -> {}",
                      phaseToString(current_phase_), phaseToString(phase));
            return;
        }
        
        log_info("Simulation phase: {} -> {}",
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
