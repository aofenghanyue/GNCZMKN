#pragma once

#include "gnc/bootstrap/install_builtin_services.hpp"
#include "gnc/common/string_utils.hpp"
#include "gnc/core/assembly_descriptor.hpp"
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
        selected_form_family_.clear();
        assembly_descriptors_.clear();
    }

    void installGlobalServices(const ConfigNode& global_service_config) {
        buildServices(global_service_config, global_services_, "global", "");
    }

    void buildMission(const ConfigNode& root) {
        buildEnvironment(root["environment"]);
        prepareVehicle(root["vehicle"]);
        buildForm(root["form"]);
        buildVehicleGroups(root["vehicle"]);
        buildInteraction(root["interaction"]);
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

    const std::vector<AssemblyDescriptor>& getAssemblyDescriptors() const {
        return assembly_descriptors_;
    }

    const std::string& getSelectedFormFamily() const {
        return selected_form_family_;
    }

private:
    struct PlacementSpec {
        std::string context;
        std::string placement;
        std::string name_prefix;
        ComponentPackageRole expected_role = ComponentPackageRole::Unknown;
        ExecutionStage execution_stage = ExecutionStage::None;
        ServiceContext* local_services = nullptr;
        std::vector<ComponentBase*>* owner_components = nullptr;
    };

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

    void buildEnvironment(const ConfigNode& environment_config) {
        if (environment_config.isNull()) {
            return;
        }
        if (!environment_config.isObject()) {
            add_error_("Top-level 'environment' must be an object.");
            return;
        }

        environment_.id = "environment";
        buildServices(environment_config["services"],
                      environment_.services,
                      "environment",
                      "env");

        PlacementSpec placement;
        placement.context = "environment";
        placement.placement = "environment";
        placement.name_prefix = "env.";
        placement.expected_role = ComponentPackageRole::Environment;
        placement.execution_stage = ExecutionStage::Environment;
        placement.owner_components = &environment_.components;
        registerComponents(environment_config["components"], placement);
    }

    void prepareVehicle(const ConfigNode& vehicle_config) {
        vehicles_.clear();
        vehicles_.push_back(VehicleInstance{});
        auto& vehicle = vehicles_.back();
        vehicle.id = "vehicle";

        if (vehicle_config.isNull()) {
            return;
        }
        if (!vehicle_config.isObject()) {
            add_error_("Top-level 'vehicle' must be an object.");
            return;
        }

        buildServices(vehicle_config["services"],
                      vehicle.services,
                      "vehicle",
                      "vehicle");
    }

    void buildForm(const ConfigNode& form_config) {
        if (form_config.isNull()) {
            add_error_("Mission configuration must define a top-level 'form' object.");
            return;
        }
        if (!form_config.isObject()) {
            add_error_("Top-level 'form' must be an object.");
            return;
        }

        const std::string declared_family = form_config["family"].asString();
        if (!declared_family.empty()) {
            selected_form_family_ = declared_family;
        }

        PlacementSpec placement;
        placement.context = "form";
        placement.placement = "form";
        placement.name_prefix = "vehicle.";
        placement.expected_role = ComponentPackageRole::Form;
        placement.execution_stage = ExecutionStage::Form;
        placement.local_services = vehicles_.empty() ? nullptr : &vehicles_.back().services;
        placement.owner_components =
            vehicles_.empty() ? nullptr : &vehicles_.back().components;
        registerComponents(form_config["components"], placement);
    }

    void buildVehicleGroups(const ConfigNode& vehicle_config) {
        if (vehicle_config.isNull()) {
            return;
        }
        if (!vehicle_config.isObject()) {
            return;
        }

        auto* vehicle_services =
            vehicles_.empty() ? nullptr : &vehicles_.back().services;
        auto* vehicle_components =
            vehicles_.empty() ? nullptr : &vehicles_.back().components;

        registerComponents(
            vehicle_config["common"],
            PlacementSpec{"vehicle.common",
                          "vehicle.common",
                          "vehicle.",
                          ComponentPackageRole::VehicleCommon,
                          ExecutionStage::None,
                          vehicle_services,
                          vehicle_components});
        registerComponents(
            vehicle_config["input"],
            PlacementSpec{"vehicle.input",
                          "vehicle.input",
                          "vehicle.",
                          ComponentPackageRole::VehicleInput,
                          ExecutionStage::VehicleInput,
                          vehicle_services,
                          vehicle_components});
        registerComponents(
            vehicle_config["process"],
            PlacementSpec{"vehicle.process",
                          "vehicle.process",
                          "vehicle.",
                          ComponentPackageRole::VehicleProcess,
                          ExecutionStage::VehicleProcess,
                          vehicle_services,
                          vehicle_components});
        registerComponents(
            vehicle_config["output"],
            PlacementSpec{"vehicle.output",
                          "vehicle.output",
                          "vehicle.",
                          ComponentPackageRole::VehicleOutput,
                          ExecutionStage::VehicleOutput,
                          vehicle_services,
                          vehicle_components});
    }

    void buildInteraction(const ConfigNode& interaction_config) {
        if (interaction_config.isNull()) {
            return;
        }
        if (!interaction_config.isObject()) {
            add_error_("Top-level 'interaction' must be an object.");
            return;
        }

        auto* vehicle_services =
            vehicles_.empty() ? nullptr : &vehicles_.back().services;
        auto* vehicle_components =
            vehicles_.empty() ? nullptr : &vehicles_.back().components;

        registerComponents(
            interaction_config["components"],
            PlacementSpec{"interaction",
                          "interaction",
                          "vehicle.",
                          ComponentPackageRole::Interaction,
                          ExecutionStage::Interaction,
                          vehicle_services,
                          vehicle_components});
    }

    void validatePlacement(const std::string& type_name,
                           const std::string& full_name,
                           const PlacementSpec& placement,
                           const ComponentFactory& factory) {
        const auto registered_role = factory.getPackageRole(type_name);
        if (placement.expected_role != ComponentPackageRole::Unknown &&
            registered_role != ComponentPackageRole::Unknown &&
            registered_role != placement.expected_role) {
            add_error_("Component '" + full_name + "' of type '" + type_name +
                       "' is registered as role '" + toString(registered_role) +
                       "' but was placed in '" + placement.placement + "'.");
        }

        const auto registered_stage = factory.getExecutionStage(type_name);
        if (placement.execution_stage == ExecutionStage::None &&
            registered_stage != ExecutionStage::None) {
            add_error_("Component '" + full_name + "' of type '" + type_name +
                       "' is registered for stage '" + toString(registered_stage) +
                       "' but was placed in '" + placement.placement +
                       "' which is non-scheduled and requires execution stage 'none'.");
        } else if (placement.execution_stage != ExecutionStage::None &&
                   registered_stage != ExecutionStage::None &&
                   registered_stage != placement.execution_stage) {
            add_error_("Component '" + full_name + "' of type '" + type_name +
                       "' is registered for stage '" + toString(registered_stage) +
                       "' but was placed in '" + placement.placement +
                       "' which executes at stage '" +
                       toString(placement.execution_stage) + "'.");
        }

        const std::string type_form_family = factory.getFormFamily(type_name);
        if (placement.placement == "form") {
            if (!type_form_family.empty()) {
                if (selected_form_family_.empty()) {
                    selected_form_family_ = type_form_family;
                } else if (selected_form_family_ != type_form_family) {
                    add_error_("Form component '" + full_name + "' of type '" + type_name +
                               "' advertises form family '" + type_form_family +
                               "' but the selected form family is '" +
                               selected_form_family_ + "'.");
                }
            }
            return;
        }

        if (!selected_form_family_.empty() && !type_form_family.empty() &&
            type_form_family != selected_form_family_) {
            add_error_("Component '" + full_name + "' of type '" + type_name +
                       "' targets form family '" + type_form_family +
                       "' but the selected form family is '" +
                       selected_form_family_ + "'.");
        }
    }

    void registerComponents(const ConfigNode& components,
                            const PlacementSpec& placement) {
        if (components.isNull()) {
            return;
        }
        if (!components.isArray()) {
            add_error_("Component block '" + placement.context + "' must be an array.");
            return;
        }

        auto& registry = simulator_.getRegistry();
        auto& factory = ComponentFactory::instance();

        for (size_t i = 0; i < components.size(); ++i) {
            const auto& component_config = components[i];
            const std::string type_name = component_config["type"].asString();
            const std::string base_name = component_config["name"].asString();
            const std::string full_name = placement.name_prefix + base_name;

            if (type_name.empty() || base_name.empty()) {
                add_error_("Component at index " + std::to_string(i) +
                           " in " + placement.context +
                           " is missing type or name.");
                continue;
            }
            if (!factory.hasType(type_name)) {
                add_error_(buildUnknownTypeMessage(type_name,
                                                   full_name,
                                                   factory,
                                                   placement.context));
                continue;
            }
            if (registry.has(full_name)) {
                add_error_("Duplicate component name '" + full_name + "'.");
                continue;
            }

            validatePlacement(type_name, full_name, placement, factory);

            auto component = factory.create(type_name);
            auto* component_ptr = component.get();
            annotateComponentMetadata(component_ptr, type_name, factory);

            const auto& config = component_config["config"];
            config.resetAccessTracking();
            component_ptr->configure(config);
            checkUnusedConfigKeys(full_name, type_name, config);

            component_ptr->injectServices(global_services_);
            component_ptr->injectServices(environment_.services);
            if (placement.local_services) {
                component_ptr->injectServices(*placement.local_services);
            }

            if (placement.owner_components) {
                placement.owner_components->push_back(component_ptr);
            }

            simulator_.addComponentToStage(component_ptr, placement.execution_stage);
            assembly_descriptors_.push_back(AssemblyDescriptor{
                full_name,
                type_name,
                placement.placement,
                factory.getPackageRole(type_name),
                factory.getExecutionStage(type_name),
                factory.getFormFamily(type_name)});

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
                add_error_("Service '" + service_name + "' in scope '" +
                           service_scope_name + "' failed to install: " + e.what());
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
    std::string selected_form_family_;
    std::vector<AssemblyDescriptor> assembly_descriptors_;
};

} // namespace gnc::core
