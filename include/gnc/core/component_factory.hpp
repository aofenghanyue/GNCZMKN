/**
 * @file component_factory.hpp
 * @brief 组件工厂
 * 
 * 支持通过字符串名称创建组件，同时保持类型安全
 */
#pragma once

#include "component_base.hpp"
#include "gnc/common/logger.hpp"
#include <memory>
#include <string>
#include <unordered_map>
#include <functional>
#include <stdexcept>
#include <vector>
#include <typeindex>

namespace gnc::core {

/**
 * @brief 组件创建器基类
 */
class ComponentCreatorBase {
public:
    virtual ~ComponentCreatorBase() = default;
    virtual std::unique_ptr<ComponentBase> create() const = 0;
    virtual std::vector<std::type_index> getInterfaces() const = 0;
};

/**
 * @brief 类型化组件创建器
 * @tparam T 组件类型
 * @tparam Interfaces 组件实现的接口列表
 */
template<typename T, typename... Interfaces>
class ComponentCreator : public ComponentCreatorBase {
public:
    std::unique_ptr<ComponentBase> create() const override {
        return std::make_unique<T>();
    }
    
    std::vector<std::type_index> getInterfaces() const override {
        return {std::type_index(typeid(Interfaces))...};
    }
};

/**
 * @brief 组件工厂
 * 
 * 允许通过字符串名称创建组件，用于配置文件驱动的仿真设置。
 * 
 * 使用示例：
 * @code
 * // 注册组件类型（通常在程序启动时）
 * ComponentFactory& factory = ComponentFactory::instance();
 * factory.registerType<IdealImu, IImuSensor>("IdealImu");
 * factory.registerType<SimpleDynamics, IDynamics>("SimpleDynamics");
 * 
 * // 从配置创建组件
 * auto imu = factory.create("IdealImu");  // 返回 unique_ptr<ComponentBase>
 * 
 * // 获取接口信息（用于自动注册到Registry）
 * auto interfaces = factory.getInterfaces("IdealImu");
 * @endcode
 */
class ComponentFactory {
public:
    /// 获取单例实例
    static ComponentFactory& instance() {
        static ComponentFactory factory;
        return factory;
    }
    
    /**
     * @brief 注册组件类型
     * @tparam T 组件类型
     * @tparam Interfaces 组件实现的接口列表
     * @param type_name 类型名称（用于配置文件）
     */
    template<typename T, typename... Interfaces>
    void registerType(const std::string& type_name) {
        if (creators_.count(type_name) > 0) {
            log_warning("Component type already registered: {}", type_name);
            return;
        }
        creators_[type_name] = std::make_unique<ComponentCreator<T, Interfaces...>>();
        log_info("Factory registered type: {}", type_name);
    }
    
    /**
     * @brief 创建组件实例
     * @param type_name 类型名称
     * @return 组件实例
     * @throws std::runtime_error 如果类型未注册
     */
    std::unique_ptr<ComponentBase> create(const std::string& type_name) const {
        auto it = creators_.find(type_name);
        if (it == creators_.end()) {
            throw std::runtime_error("Unknown component type: " + type_name);
        }
        return it->second->create();
    }
    
    /**
     * @brief 获取组件类型实现的接口列表
     * @param type_name 类型名称
     * @return 接口类型索引列表
     */
    std::vector<std::type_index> getInterfaces(const std::string& type_name) const {
        auto it = creators_.find(type_name);
        if (it == creators_.end()) {
            return {};
        }
        return it->second->getInterfaces();
    }
    
    /**
     * @brief 检查类型是否已注册
     */
    bool hasType(const std::string& type_name) const {
        return creators_.count(type_name) > 0;
    }
    
    /**
     * @brief 获取所有注册的类型名称
     */
    std::vector<std::string> getRegisteredTypes() const {
        std::vector<std::string> types;
        for (const auto& [name, _] : creators_) {
            types.push_back(name);
        }
        return types;
    }
    
private:
    ComponentFactory() = default;
    std::unordered_map<std::string, std::unique_ptr<ComponentCreatorBase>> creators_;
};

/**
 * @brief 自动注册宏
 * 
 * 在组件头文件末尾使用，自动注册到工厂
 */
#define GNC_REGISTER_COMPONENT(ComponentType, ...) \
    namespace { \
        struct ComponentType##_Registrar { \
            ComponentType##_Registrar() { \
                gnc::core::ComponentFactory::instance() \
                    .registerType<ComponentType, __VA_ARGS__>(#ComponentType); \
            } \
        } ComponentType##_registrar_instance; \
    }

} // namespace gnc::core
