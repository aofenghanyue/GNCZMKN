/**
 * @file component_registry.hpp
 * @brief 组件注册表
 * 
 * 支持：
 * - 命名注册：同类型多实例
 * - 接口获取：按接口类型查找组件
 */
#pragma once

#include "component_base.hpp"
#include "gnc/common/logger.hpp"
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <typeindex>
#include <stdexcept>

namespace gnc::core {

/**
 * @brief 组件注册表
 * 
 * 管理所有组件的注册和查找，支持按名称和接口类型获取组件
 */
class ComponentRegistry {
public:
    ComponentRegistry() = default;
    ~ComponentRegistry() = default;
    
    // 禁止拷贝
    ComponentRegistry(const ComponentRegistry&) = delete;
    ComponentRegistry& operator=(const ComponentRegistry&) = delete;
    
    /**
     * @brief 注册组件
     * @tparam T 组件类型
     * @tparam Interfaces 组件实现的接口类型列表
     * @param name 组件唯一名称
     * @param component 组件指针
     */
    template<typename T, typename... Interfaces>
    void add(const std::string& name, std::unique_ptr<T> component) {
        if (components_.count(name) > 0) {
            throw std::runtime_error("Component already registered: " + name);
        }
        
        T* raw_ptr = component.get();
        components_[name] = std::move(component);
        
        // 注册所有接口
        registerInterfaces<T, Interfaces...>(name, raw_ptr);
        
        // 记录组件顺序
        component_order_.push_back(name);
        
        LOG_INFO("Registered component: {}", name);
    }
    
    /**
     * @brief 按名称获取组件
     * @tparam T 期望的类型（组件类或接口类）
     * @param name 组件名称
     * @return T* 组件指针，找不到返回nullptr
     */
    template<typename T>
    T* get(const std::string& name) const {
        auto it = components_.find(name);
        if (it == components_.end()) {
            return nullptr;
        }
        return dynamic_cast<T*>(it->second.get());
    }
    
    
    
    /**
     * @brief 获取实现特定接口的所有组件
     * @tparam Interface 接口类型
     * @return 接口指针列表
     */
    template<typename Interface>
    std::vector<Interface*> getAll() const {
        std::vector<Interface*> result;
        auto type_idx = std::type_index(typeid(Interface));
        auto it = interface_map_.find(type_idx);
        if (it != interface_map_.end()) {
            for (const auto& entry : it->second) {
                auto* casted = dynamic_cast<Interface*>(entry.ptr);
                if (casted) {
                    result.push_back(casted);
                }
            }
        }
        return result;
    }
    
    /**
     * @brief 按注册顺序获取所有组件
     */
    std::vector<ComponentBase*> getAllComponents() const {
        std::vector<ComponentBase*> result;
        for (const auto& name : component_order_) {
            result.push_back(components_.at(name).get());
        }
        return result;
    }
    
    /**
     * @brief 检查组件是否存在
     */
    bool has(const std::string& name) const {
        return components_.count(name) > 0;
    }
    
    /**
     * @brief 获取组件数量
     */
    size_t size() const { return components_.size(); }
    
    /**
     * @brief 动态注册组件（运行时接口注册）
     * 用于工厂创建的组件
     */
    void addDynamic(const std::string& name, 
                    std::unique_ptr<ComponentBase> component,
                    const std::vector<std::type_index>& interfaces) {
        if (components_.count(name) > 0) {
            throw std::runtime_error("Component already registered: " + name);
        }
        
        ComponentBase* raw_ptr = component.get();
        components_[name] = std::move(component);
        
        // 动态注册接口
        for (const auto& type_idx : interfaces) {
            interface_map_[type_idx].push_back({name, raw_ptr});
        }
        
        component_order_.push_back(name);
        LOG_INFO("Registered component (dynamic): {}", name);
    }
    
private:
    struct InterfaceEntry {
        std::string name;
        ComponentBase* ptr;  // stored as base ptr; getAll() uses dynamic_cast
    };
    
    // 递归注册接口
    template<typename T>
    void registerInterfaces(const std::string&, T*) {
        // 递归终止
    }
    
    template<typename T, typename First, typename... Rest>
    void registerInterfaces(const std::string& name, T* ptr) {
        auto type_idx = std::type_index(typeid(First));
        // Store as ComponentBase*; getAll() will dynamic_cast to the correct interface
        interface_map_[type_idx].push_back({name, static_cast<ComponentBase*>(ptr)});
        registerInterfaces<T, Rest...>(name, ptr);
    }
    
    std::unordered_map<std::string, std::unique_ptr<ComponentBase>> components_;
    std::unordered_map<std::type_index, std::vector<InterfaceEntry>> interface_map_;
    std::vector<std::string> component_order_;
};

} // namespace gnc::core
