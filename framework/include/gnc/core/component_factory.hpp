#pragma once

#include "component_base.hpp"
#include "gnc/common/logger.hpp"

#include <algorithm>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <vector>

namespace gnc::core {

enum class ComponentCategory {
    Builtin,
    Project
};

inline const char* toString(ComponentCategory category) {
    switch (category) {
    case ComponentCategory::Builtin:
        return "builtin";
    case ComponentCategory::Project:
        return "project";
    default:
        return "unknown";
    }
}

inline std::string normalizeRegistrationOrigin(std::string path) {
    std::replace(path.begin(), path.end(), '\\', '/');
    return path;
}

class ComponentCreatorBase {
public:
    virtual ~ComponentCreatorBase() = default;
    virtual std::unique_ptr<ComponentBase> create() const = 0;
    virtual std::vector<std::type_index> getInterfaces() const = 0;
};

template<typename T, typename... Interfaces>
class ComponentCreator final : public ComponentCreatorBase {
public:
    std::unique_ptr<ComponentBase> create() const override {
        return std::make_unique<T>();
    }

    std::vector<std::type_index> getInterfaces() const override {
        return {std::type_index(typeid(Interfaces))...};
    }
};

class ComponentFactory {
public:
    struct RegisteredTypeInfo {
        std::string type_name;
        ComponentCategory category = ComponentCategory::Project;
        std::string registration_origin;
        std::vector<std::type_index> interfaces;
    };

    static ComponentFactory& instance() {
        static ComponentFactory factory;
        return factory;
    }

    template<typename T, typename... Interfaces>
    void registerType(const std::string& type_name,
                      ComponentCategory category = ComponentCategory::Project,
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

    std::unique_ptr<ComponentBase> create(const std::string& type_name) const {
        const auto it = creators_.find(type_name);
        if (it == creators_.end()) {
            throw std::runtime_error(
                "Unknown component type: '" + type_name +
                "'. Available registered types: " + describeRegisteredTypes());
        }
        return it->second.creator->create();
    }

    std::vector<std::type_index> getInterfaces(const std::string& type_name) const {
        const auto it = creators_.find(type_name);
        if (it == creators_.end()) {
            return {};
        }
        return it->second.creator->getInterfaces();
    }

    bool hasType(const std::string& type_name) const {
        return creators_.count(type_name) > 0;
    }

    std::vector<std::string> getRegisteredTypes() const {
        std::vector<std::string> types;
        const auto infos = getRegisteredTypeInfos();
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

        std::sort(infos.begin(),
                  infos.end(),
                  [](const RegisteredTypeInfo& lhs, const RegisteredTypeInfo& rhs) {
                      if (lhs.category != rhs.category) {
                          return lhs.category < rhs.category;
                      }
                      return lhs.type_name < rhs.type_name;
                  });
        return infos;
    }

    std::string describeRegisteredTypes() const {
        const auto infos = getRegisteredTypeInfos();
        std::vector<std::string> builtin_types;
        std::vector<std::string> project_types;

        for (const auto& info : infos) {
            if (info.category == ComponentCategory::Builtin) {
                builtin_types.push_back(info.type_name);
            } else {
                project_types.push_back(info.type_name);
            }
        }

        auto join = [](const std::vector<std::string>& values) {
            std::string result;
            for (size_t i = 0; i < values.size(); ++i) {
                if (i > 0) {
                    result += ", ";
                }
                result += values[i];
            }
            return result.empty() ? "(none)" : result;
        };

        return "builtin=[" + join(builtin_types) + "], project=[" +
               join(project_types) + "]";
    }

    ComponentCategory getCategory(const std::string& type_name) const {
        const auto it = creators_.find(type_name);
        if (it == creators_.end()) {
            return ComponentCategory::Project;
        }
        return it->second.category;
    }

    std::string getRegistrationOrigin(const std::string& type_name) const {
        const auto it = creators_.find(type_name);
        if (it == creators_.end()) {
            return "";
        }
        return it->second.registration_origin;
    }

private:
    ComponentFactory() = default;

    struct RegisteredTypeEntry {
        std::unique_ptr<ComponentCreatorBase> creator;
        ComponentCategory category = ComponentCategory::Project;
        std::string registration_origin;
    };

    std::unordered_map<std::string, RegisteredTypeEntry> creators_;
};

#define GNC_REGISTER_COMPONENT_TYPE(TypeId, ComponentType, ...) \
    namespace { \
        struct ComponentType##_Registrar { \
            ComponentType##_Registrar() { \
                ::gnc::core::ComponentFactory::instance() \
                    .registerType<ComponentType, __VA_ARGS__>( \
                        TypeId, \
                        ::gnc::core::ComponentCategory::Project, \
                        __FILE__); \
            } \
        } ComponentType##_registrar_instance; \
    }

#define GNC_REGISTER_BUILTIN_COMPONENT(TypeId, ComponentType, ...) \
    namespace { \
        struct ComponentType##_BuiltinRegistrar { \
            ComponentType##_BuiltinRegistrar() { \
                ::gnc::core::ComponentFactory::instance() \
                    .registerType<ComponentType, __VA_ARGS__>( \
                        TypeId, \
                        ::gnc::core::ComponentCategory::Builtin, \
                        __FILE__); \
            } \
        } ComponentType##_builtin_registrar_instance; \
    }

} // namespace gnc::core
