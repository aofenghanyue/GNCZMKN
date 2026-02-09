/**
 * @file scoped_registry.hpp
 * @brief 作用域组件注册表视图
 * 
 * 为每个飞行器/环境提供隔离的组件访问视图
 * 组件只需使用简单名称即可访问同作用域内的其他组件
 */
#pragma once

#include "component_registry.hpp"
#include <string>

namespace gnc::core {

/**
 * @brief 作用域组件视图
 * 
 * 包装 ComponentRegistry，提供飞行器/环境级别的组件隔离
 * 
 * 示例：
 * - ScopedRegistry("chaser.", registry)
 * - getByName<IImu>("imu") → 解析为 "chaser.imu"
 * - getByName<ITime>("env.time") → 直接解析为 "env.time"（全名访问）
 */
class ScopedRegistry {
public:
    /**
     * @brief 构造作用域视图
     * @param scope 作用域前缀（如 "chaser." 或 "env."），应以 '.' 结尾
     * @param registry 全局组件注册表引用
     */
    ScopedRegistry(const std::string& scope, ComponentRegistry& registry)
        : scope_(scope), registry_(registry) {}
    
    /**
     * @brief 按名称获取接口实现
     * 
     * 解析顺序：
     * 1. 如果 name 包含 '.'，视为全名直接查找
     * 2. 否则，添加 scope 前缀后查找
     * 
     * @tparam Interface 接口类型
     * @param name 组件名称（简单名或全名）
     * @return Interface* 接口指针，找不到返回 nullptr
     */
    template<typename Interface>
    Interface* getByName(const std::string& name) const {
        // 包含 '.' 视为全名
        if (name.find('.') != std::string::npos) {
            return registry_.get<Interface>(name);
        }
        
        // 添加作用域前缀
        std::string fullName = scope_ + name;
        return registry_.get<Interface>(fullName);
    }
    
    /**
     * @brief 获取本作用域内实现指定接口的所有组件
     * 
     * 只返回名称以当前 scope 为前缀的组件
     * 
     * @tparam Interface 接口类型
     * @return 接口指针列表
     */
    template<typename Interface>
    std::vector<Interface*> getAll() const {
        std::vector<Interface*> result;
        auto all = registry_.getAll<Interface>();
        
        for (auto* ptr : all) {
            // 需要通过 registry 反查名称来过滤
            // 由于当前 registry 不支持反查，返回所有实现
            // TODO: 后续可优化为只返回本作用域内的组件
            result.push_back(ptr);
        }
        
        return result;
    }
    
    /// 获取当前作用域前缀
    const std::string& getScope() const { return scope_; }
    
    /// 获取底层 registry（供高级用法）
    ComponentRegistry& getRegistry() { return registry_; }
    const ComponentRegistry& getRegistry() const { return registry_; }
    
private:
    std::string scope_;           // 作用域前缀，如 "chaser." 或 "env."
    ComponentRegistry& registry_; // 全局注册表引用
};

} // namespace gnc::core
