#pragma once

#include "component_base.hpp"
#include "gnc/common/logger.hpp"

#include <algorithm>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <typeindex>
#include <unordered_map>
#include <vector>

namespace gnc::core {

enum class ComponentCategory {
    Builtin,
    Project
};

enum class ComponentPackageRole {
    Unknown,
    Form,
    Environment,
    VehicleCommon,
    VehicleInput,
    VehicleProcess,
    VehicleOutput,
    Interaction,
    Termination,
    Summary
};

enum class ExecutionStage {
    None,
    Environment,
    VehicleInput,
    VehicleProcess,
    VehicleOutput,
    Interaction,
    Form,
    Termination,
    Summary
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

inline const char* toString(ComponentPackageRole role) {
    switch (role) {
    case ComponentPackageRole::Unknown:
        return "unknown";
    case ComponentPackageRole::Form:
        return "form";
    case ComponentPackageRole::Environment:
        return "environment";
    case ComponentPackageRole::VehicleCommon:
        return "vehicle_common";
    case ComponentPackageRole::VehicleInput:
        return "vehicle_input";
    case ComponentPackageRole::VehicleProcess:
        return "vehicle_process";
    case ComponentPackageRole::VehicleOutput:
        return "vehicle_output";
    case ComponentPackageRole::Interaction:
        return "interaction";
    case ComponentPackageRole::Termination:
        return "termination";
    case ComponentPackageRole::Summary:
        return "summary";
    default:
        return "unknown";
    }
}

inline const char* toString(ExecutionStage stage) {
    switch (stage) {
    case ExecutionStage::None:
        return "none";
    case ExecutionStage::Environment:
        return "environment";
    case ExecutionStage::VehicleInput:
        return "vehicle_input";
    case ExecutionStage::VehicleProcess:
        return "vehicle_process";
    case ExecutionStage::VehicleOutput:
        return "vehicle_output";
    case ExecutionStage::Interaction:
        return "interaction";
    case ExecutionStage::Form:
        return "form";
    case ExecutionStage::Termination:
        return "termination";
    case ExecutionStage::Summary:
        return "summary";
    default:
        return "none";
    }
}

inline std::string normalizeRegistrationOrigin(std::string path) {
    std::replace(path.begin(), path.end(), '\\', '/');
    return path;
}

inline void stripTypePrefix(std::string_view& value) {
    constexpr std::string_view class_prefix = "class ";
    constexpr std::string_view struct_prefix = "struct ";
    while (value.substr(0, class_prefix.size()) == class_prefix ||
           value.substr(0, struct_prefix.size()) == struct_prefix) {
        if (value.substr(0, class_prefix.size()) == class_prefix) {
            value.remove_prefix(class_prefix.size());
            continue;
        }
        value.remove_prefix(struct_prefix.size());
    }
}

template<typename T>
std::string readableTypeName() {
#if defined(__clang__) || defined(__GNUC__)
    std::string_view function = __PRETTY_FUNCTION__;
    constexpr std::string_view marker = "T = ";
    const auto start = function.find(marker);
    if (start == std::string_view::npos) {
        return typeid(T).name();
    }
    std::string_view value = function.substr(start + marker.size());
    const auto end = value.find_first_of(";]");
    if (end != std::string_view::npos) {
        value = value.substr(0, end);
    }
#elif defined(_MSC_VER)
    std::string_view function = __FUNCSIG__;
    constexpr std::string_view prefix = "readableTypeName<";
    const auto start = function.find(prefix);
    if (start == std::string_view::npos) {
        return typeid(T).name();
    }
    std::string_view value = function.substr(start + prefix.size());
    const auto end = value.find(">(void)");
    if (end != std::string_view::npos) {
        value = value.substr(0, end);
    }
#else
    return typeid(T).name();
#endif

    stripTypePrefix(value);
    const auto namespace_pos = value.rfind("::");
    if (namespace_pos != std::string_view::npos) {
        value.remove_prefix(namespace_pos + 2);
    }
    return std::string(value);
}

class ComponentCreatorBase {
public:
    virtual ~ComponentCreatorBase() = default;
    virtual std::unique_ptr<ComponentBase> create() const = 0;
    virtual std::vector<std::type_index> getInterfaces() const = 0;
    virtual std::vector<std::string> getInterfaceNames() const = 0;
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

    std::vector<std::string> getInterfaceNames() const override {
        return {readableTypeName<Interfaces>()...};
    }
};

class ComponentFactory {
public:
    struct RegisteredTypeInfo {
        std::string type_name;
        ComponentCategory category = ComponentCategory::Project;
        std::string registration_origin;
        ComponentPackageRole package_role = ComponentPackageRole::Unknown;
        ExecutionStage execution_stage = ExecutionStage::None;
        std::string form_family;
        std::vector<std::type_index> interfaces;
        std::vector<std::string> interface_names;
    };

    static ComponentFactory& instance() {
        static ComponentFactory factory;
        return factory;
    }

    template<typename T, typename... Interfaces>
    void registerType(const std::string& type_name,
                      ComponentCategory category = ComponentCategory::Project,
                      const std::string& registration_origin = "",
                      ComponentPackageRole package_role = ComponentPackageRole::Unknown,
                      ExecutionStage execution_stage = ExecutionStage::None,
                      const std::string& form_family = "") {
        if (creators_.count(type_name) > 0) {
            LOG_WARNING("Component type already registered: {}", type_name);
            return;
        }

        RegisteredTypeEntry entry;
        entry.creator = std::make_unique<ComponentCreator<T, Interfaces...>>();
        entry.category = category;
        entry.registration_origin = normalizeRegistrationOrigin(registration_origin);
        entry.package_role = package_role;
        entry.execution_stage = execution_stage;
        entry.form_family = form_family;
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

    std::vector<std::string> getInterfaceNames(const std::string& type_name) const {
        const auto it = creators_.find(type_name);
        if (it == creators_.end()) {
            return {};
        }
        return it->second.creator->getInterfaceNames();
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
            info.package_role = entry.package_role;
            info.execution_stage = entry.execution_stage;
            info.form_family = entry.form_family;
            info.interfaces = entry.creator->getInterfaces();
            info.interface_names = entry.creator->getInterfaceNames();
            std::sort(info.interface_names.begin(), info.interface_names.end());
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

    ComponentPackageRole getPackageRole(const std::string& type_name) const {
        const auto it = creators_.find(type_name);
        if (it == creators_.end()) {
            return ComponentPackageRole::Unknown;
        }
        return it->second.package_role;
    }

    ExecutionStage getExecutionStage(const std::string& type_name) const {
        const auto it = creators_.find(type_name);
        if (it == creators_.end()) {
            return ExecutionStage::None;
        }
        return it->second.execution_stage;
    }

    std::string getFormFamily(const std::string& type_name) const {
        const auto it = creators_.find(type_name);
        if (it == creators_.end()) {
            return "";
        }
        return it->second.form_family;
    }

private:
    ComponentFactory() = default;

    struct RegisteredTypeEntry {
        std::unique_ptr<ComponentCreatorBase> creator;
        ComponentCategory category = ComponentCategory::Project;
        std::string registration_origin;
        ComponentPackageRole package_role = ComponentPackageRole::Unknown;
        ExecutionStage execution_stage = ExecutionStage::None;
        std::string form_family;
    };

    std::unordered_map<std::string, RegisteredTypeEntry> creators_;
};

#ifdef GNC_COMPONENT_REGISTRATION_FN
#define GNC_REGISTER_COMPONENT_TYPE_IMPL(TypeId, \
                                         ComponentType, \
                                         Category, \
                                         PackageRole, \
                                         ExecutionStage, \
                                         FormFamily, \
                                         ...) \
    inline void GNC_COMPONENT_REGISTRATION_FN(::gnc::core::ComponentFactory& factory) { \
        factory.registerType<ComponentType, __VA_ARGS__>( \
            TypeId, \
            Category, \
            __FILE__, \
            PackageRole, \
            ExecutionStage, \
            FormFamily); \
    }
#else
#define GNC_REGISTER_COMPONENT_TYPE_IMPL(TypeId, \
                                         ComponentType, \
                                         Category, \
                                         PackageRole, \
                                         ExecutionStage, \
                                         FormFamily, \
                                         ...) \
    static_assert(false, \
                  "GNC_REGISTER_COMPONENT_TYPE requires GNC_COMPONENT_REGISTRATION_FN " \
                  "to be defined by the build-generated explicit registration chain, " \
                  "and project registrations must declare package role, execution stage, " \
                  "and form family metadata.")
#endif

#define GNC_REGISTER_COMPONENT_TYPE(TypeId, \
                                    ComponentType, \
                                    PackageRole, \
                                    ExecutionStage, \
                                    FormFamily, \
                                    ...) \
    GNC_REGISTER_COMPONENT_TYPE_IMPL(TypeId, \
                                     ComponentType, \
                                     ::gnc::core::ComponentCategory::Project, \
                                     PackageRole, \
                                     ExecutionStage, \
                                     FormFamily, \
                                     __VA_ARGS__)

#define GNC_REGISTER_BUILTIN_COMPONENT(TypeId, \
                                       ComponentType, \
                                       PackageRole, \
                                       ExecutionStage, \
                                       FormFamily, \
                                       ...) \
    GNC_REGISTER_COMPONENT_TYPE_IMPL(TypeId, \
                                     ComponentType, \
                                     ::gnc::core::ComponentCategory::Builtin, \
                                     PackageRole, \
                                     ExecutionStage, \
                                     FormFamily, \
                                     __VA_ARGS__)

} // namespace gnc::core
