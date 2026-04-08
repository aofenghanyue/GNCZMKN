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
#include <algorithm>

namespace gnc::core {

enum class ComponentCategory {
    Starter,
    Custom
};

inline const char* toString(ComponentCategory category) {
    switch (category) {
    case ComponentCategory::Starter:
        return "starter";
    case ComponentCategory::Custom:
        return "custom";
    default:
        return "unknown";
    }
}

inline std::string normalizeRegistrationOrigin(std::string path) {
    std::replace(path.begin(), path.end(), '\\', '/');
    return path;
}

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
    struct RegisteredTypeInfo {
        std::string type_name;
        ComponentCategory category = ComponentCategory::Custom;
        std::string registration_origin;
        std::vector<std::type_index> interfaces;
    };

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
    void registerType(const std::string& type_name,
                      ComponentCategory category = ComponentCategory::Custom,
                      const std::string& registration_origin = "") {
        if (creators_.count(type_name) > 0) {
            LOG_WARNING("Component type already registered: {}", type_name);
            return;
        }
        RegisteredTypeEntry entry;
        entry.creator = std::make_unique<ComponentCreator<T, Interfaces...>>();
        entry.category = category;
        entry.registration_origin = normalizeRegistrationOrigin(registration_origin);
        creators_[type_name] = std::move(entry);
        LOG_INFO("Factory registered {} type: {}", toString(category), type_name);
    }

    template<typename T, typename... Interfaces>
    void registerStarterType(const std::string& type_name,
                             const std::string& registration_origin = "") {
        registerType<T, Interfaces...>(type_name, ComponentCategory::Starter, registration_origin);
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
            std::string msg = "Unknown component type: '" + type_name +
                              "'. Available registered types: " + describeRegisteredTypes();
            throw std::runtime_error(msg);
        }
        return it->second.creator->create();
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
        return it->second.creator->getInterfaces();
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
        auto infos = getRegisteredTypeInfos();
        types.reserve(infos.size());
        for (const auto& info : infos) {
            types.push_back(info.type_name);
        }
        return types;
    }

    std::vector<RegisteredTypeInfo> getRegisteredTypeInfos() const {
        std::vector<RegisteredTypeInfo> infos;
        infos.reserve(creators_.size());
        for (const auto& [name, entry] : creators_) {
            RegisteredTypeInfo info;
            info.type_name = name;
            info.category = entry.category;
            info.registration_origin = entry.registration_origin;
            info.interfaces = entry.creator->getInterfaces();
            infos.push_back(std::move(info));
        }

        std::sort(infos.begin(), infos.end(), [](const RegisteredTypeInfo& lhs,
                                                 const RegisteredTypeInfo& rhs) {
            if (lhs.category != rhs.category) {
                return lhs.category < rhs.category;
            }
            return lhs.type_name < rhs.type_name;
        });
        return infos;
    }

    std::string describeRegisteredTypes() const {
        const auto infos = getRegisteredTypeInfos();
        std::vector<std::string> starter_types;
        std::vector<std::string> custom_types;

        for (const auto& info : infos) {
            if (info.category == ComponentCategory::Starter) {
                starter_types.push_back(info.type_name);
            } else {
                custom_types.push_back(info.type_name);
            }
        }

        auto join = [](const std::vector<std::string>& values) {
            std::string result;
            for (size_t i = 0; i < values.size(); ++i) {
                if (i > 0) result += ", ";
                result += values[i];
            }
            return result.empty() ? "(none)" : result;
        };

        return "starter=[" + join(starter_types) + "], custom=[" + join(custom_types) + "]";
    }

    ComponentCategory getCategory(const std::string& type_name) const {
        auto it = creators_.find(type_name);
        if (it == creators_.end()) {
            return ComponentCategory::Custom;
        }
        return it->second.category;
    }

    std::string getRegistrationOrigin(const std::string& type_name) const {
        auto it = creators_.find(type_name);
        if (it == creators_.end()) {
            return "";
        }
        return it->second.registration_origin;
    }
    
private:
    ComponentFactory() = default;

    struct RegisteredTypeEntry {
        std::unique_ptr<ComponentCreatorBase> creator;
        ComponentCategory category = ComponentCategory::Custom;
        std::string registration_origin;
    };

    std::unordered_map<std::string, RegisteredTypeEntry> creators_;
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
                    .registerType<ComponentType, __VA_ARGS__>(#ComponentType, gnc::core::ComponentCategory::Custom, __FILE__); \
            } \
        } ComponentType##_registrar_instance; \
    }

#define GNC_REGISTER_STARTER_COMPONENT(ComponentType, ...) \
    namespace { \
        struct ComponentType##_StarterRegistrar { \
            ComponentType##_StarterRegistrar() { \
                gnc::core::ComponentFactory::instance() \
                    .registerStarterType<ComponentType, __VA_ARGS__>(#ComponentType, __FILE__); \
            } \
        } ComponentType##_starter_registrar_instance; \
    }

} // namespace gnc::core
