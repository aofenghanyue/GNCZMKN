#pragma once

#include "gnc/bootstrap/install_builtin_services.hpp"
#include "gnc/common/string_utils.hpp"
#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/core/deferred_registry_action.hpp"
#include "gnc/core/service_context.hpp"
#include "gnc/core/simulator.hpp"

#include <algorithm>
#include <exception>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace gnc::core {

struct EntityInstance {
    std::string id;
    ServiceContext services;
    std::vector<ComponentBase*> components;
};

struct VehicleInstance : EntityInstance {
};

struct EnvironmentInstance : EntityInstance {
};

class MissionAssembler {
public:
    using DiagnosticReporter = std::function<void(const std::string&)>;

    MissionAssembler(Simulator& simulator,
                     ServiceContext& global_services,
                     EnvironmentInstance& environment,
                     std::vector<VehicleInstance>& vehicles,
                     std::vector<DeferredRegistryAction>& deferred_service_actions,
                     DiagnosticReporter add_error,
                     DiagnosticReporter add_warning)
        : simulator_(simulator),
          global_services_(global_services),
          environment_(environment),
          vehicles_(vehicles),
          deferred_service_actions_(deferred_service_actions),
          add_error_(std::move(add_error)),
          add_warning_(std::move(add_warning)) {}

    void reset() {
        deferred_service_actions_.clear();
        global_services_.clear();
        environment_ = EnvironmentInstance{};
        vehicles_.clear();
    }

    void installGlobalServices(const ConfigNode& global_service_config) {
        buildServices(global_service_config, global_services_, "global", "");
    }

    void buildEntities(const ConfigNode& entities) {
        vehicles_.reserve(entities.size());

        for (size_t i = 0; i < entities.size(); ++i) {
            const auto& entity_config = entities[i];
            if (entity_config["role"].asString("vehicle") == "environment") {
                buildEntity(entity_config, i);
            }
        }

        for (size_t i = 0; i < entities.size(); ++i) {
            const auto& entity_config = entities[i];
            if (entity_config["role"].asString("vehicle") != "environment") {
                buildEntity(entity_config, i);
            }
        }
    }

    void runDeferredServiceActions() {
        auto& registry = simulator_.getRegistry();
        for (const auto& action : deferred_service_actions_) {
            try {
                action(registry);
            } catch (const std::exception& e) {
                add_error_("Deferred service installation failed: " +
                           std::string(e.what()));
            }
        }
    }

private:
    static std::string joinStrings(const std::vector<std::string>& values) {
        std::string result;
        for (size_t i = 0; i < values.size(); ++i) {
            if (i > 0) {
                result += ", ";
            }
            result += values[i];
        }
        return result.empty() ? "(none)" : result;
    }

    static std::string buildUnknownTypeMessage(const std::string& type_name,
                                               const std::string& component_name,
                                               const ComponentFactory& factory,
                                               const std::string& context = "") {
        std::string message = "Unknown component type: '" + type_name +
                              "' (component name: '" + component_name + "')";
        if (!context.empty()) {
            message += " in " + context;
        }
        message += ". Available registered types: " + factory.describeRegisteredTypes();

        const std::string suggestion =
            common::findClosestMatch(type_name, factory.getRegisteredTypes());
        if (!suggestion.empty()) {
            message += ". Did you mean '" + suggestion + "'?";
        }
        return message;
    }

    static std::string buildUnknownServiceMessage(const std::string& service_name) {
        std::string message = "Unknown builtin service '" + service_name + "'.";
        const auto available = gnc::bootstrap::getBuiltinServiceNames();
        if (!available.empty()) {
            message += " Available services: " + joinStrings(available) + ".";
        }
        const std::string suggestion =
            common::findClosestMatch(service_name, available);
        if (!suggestion.empty()) {
            message += " Did you mean '" + suggestion + "'?";
        }
        return message;
    }

    static void annotateComponentMetadata(ComponentBase* component,
                                          const std::string& type_name,
                                          const ComponentFactory& factory) {
        if (!component) {
            return;
        }
        component->setTypeNameInternal_(type_name);
        component->setComponentCategoryInternal_(toString(factory.getCategory(type_name)));
        component->setRegistrationOriginInternal_(factory.getRegistrationOrigin(type_name));
    }

    void checkUnusedConfigKeys(const std::string& component_name,
                               const std::string& type_name,
                               const ConfigNode& config) const {
        auto unused = config.getUnusedKeys();
        if (unused.empty()) {
            return;
        }
        add_warning_("Component '" + component_name + "' (type: " + type_name +
                     ") has unrecognized config keys: " + joinStrings(unused) + ".");
    }

    void buildEntity(const ConfigNode& entity_config, size_t index) {
        if (!entity_config.isObject()) {
            add_error_("Entity at index " + std::to_string(index) +
                       " must be an object.");
            return;
        }

        const std::string id = entity_config["id"].asString();
        if (id.empty()) {
            add_error_("Entity at index " + std::to_string(index) +
                       " is missing an id.");
            return;
        }

        const std::string role = entity_config["role"].asString("vehicle");
        const auto& components = entity_config["components"];
        if (!components.isArray()) {
            add_error_("Entity '" + id + "' must define a 'components' array.");
            return;
        }

        if (role == "environment") {
            if (!environment_.id.empty()) {
                add_error_("Multiple environment entities are not supported. Existing id '" +
                           environment_.id + "', duplicate id '" + id + "'.");
                return;
            }

            environment_.id = id;
            buildServices(entity_config["services"],
                          environment_.services,
                          "environment entity '" + id + "'",
                          "env");
            registerComponents(components,
                               "environment entity '" + id + "'",
                               "env.",
                               global_services_,
                               environment_.services,
                               nullptr,
                               environment_.components);
            return;
        }

        if (role != "vehicle") {
            add_error_("Entity '" + id + "' has unsupported role '" + role + "'.");
            return;
        }

        VehicleInstance vehicle;
        vehicle.id = id;
        buildServices(entity_config["services"],
                      vehicle.services,
                      "vehicle entity '" + id + "'",
                      id);
        registerComponents(components,
                           "vehicle entity '" + id + "'",
                           id + ".",
                           global_services_,
                           environment_.services,
                           &vehicle.services,
                           vehicle.components);
        vehicles_.push_back(std::move(vehicle));
    }

    void registerComponents(const ConfigNode& components,
                            const std::string& context,
                            const std::string& name_prefix,
                            ServiceContext& global_services,
                            ServiceContext& environment_services,
                            ServiceContext* local_services,
                            std::vector<ComponentBase*>& owner_components) {
        auto& registry = simulator_.getRegistry();
        auto& factory = ComponentFactory::instance();

        for (size_t i = 0; i < components.size(); ++i) {
            const auto& component_config = components[i];
            const std::string type_name = component_config["type"].asString();
            const std::string base_name = component_config["name"].asString();
            const std::string full_name = name_prefix + base_name;

            if (type_name.empty() || base_name.empty()) {
                add_error_("Component at index " + std::to_string(i) +
                           " in " + context + " is missing type or name.");
                continue;
            }
            if (!factory.hasType(type_name)) {
                add_error_(buildUnknownTypeMessage(type_name, full_name, factory, context));
                continue;
            }
            if (registry.has(full_name)) {
                add_error_("Duplicate component name '" + full_name + "'.");
                continue;
            }

            auto component = factory.create(type_name);
            auto* component_ptr = component.get();
            annotateComponentMetadata(component_ptr, type_name, factory);

            const auto& config = component_config["config"];
            config.resetAccessTracking();
            component_ptr->configure(config);
            checkUnusedConfigKeys(full_name, type_name, config);

            component_ptr->injectServices(global_services);
            component_ptr->injectServices(environment_services);
            if (local_services) {
                component_ptr->injectServices(*local_services);
            }

            owner_components.push_back(component_ptr);
            registry.addDynamic(full_name,
                                std::move(component),
                                factory.getInterfaces(type_name));
        }
    }

    void buildServices(const ConfigNode& service_config,
                       ServiceContext& services,
                       const std::string& service_scope_name,
                       const std::string& registry_scope) {
        if (service_config.isNull()) {
            return;
        }
        if (!service_config.isObject()) {
            add_error_("Service block for scope '" + service_scope_name +
                       "' must be an object.");
            return;
        }

        for (const auto& [service_name, config] : service_config) {
            if (!config.isObject()) {
                add_error_("Service '" + service_name + "' in scope '" +
                           service_scope_name + "' must be an object.");
                continue;
            }
            if (config.has("enabled") && !config["enabled"].asBool(true)) {
                continue;
            }
            try {
                gnc::bootstrap::installBuiltinService(
                    service_name,
                    config,
                    services,
                    registry_scope,
                    deferred_service_actions_);
            } catch (const std::exception& e) {
                const auto available = gnc::bootstrap::getBuiltinServiceNames();
                if (std::find(available.begin(), available.end(), service_name) ==
                    available.end()) {
                    add_error_(buildUnknownServiceMessage(service_name));
                    continue;
                }
                add_error_("Service '" + service_name + "' in scope '" + service_scope_name +
                           "' failed to install: " + e.what());
            }
        }
    }

    Simulator& simulator_;
    ServiceContext& global_services_;
    EnvironmentInstance& environment_;
    std::vector<VehicleInstance>& vehicles_;
    std::vector<DeferredRegistryAction>& deferred_service_actions_;
    DiagnosticReporter add_error_;
    DiagnosticReporter add_warning_;
};

} // namespace gnc::core
