#pragma once

#include "gnc/common/string_utils.hpp"
#include "gnc/core/assembly_descriptor.hpp"
#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/core/service_package_registry.hpp"
#include "gnc/core/service_context.hpp"
#include "gnc/core/simulator.hpp"
#include "gnc/interfaces/i_summary_observer.hpp"
#include "gnc/interfaces/i_termination_evaluator.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
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
                     ServicePackageRegistry& service_packages,
                     double simulation_dt,
                     DiagnosticReporter add_error,
                     DiagnosticReporter add_warning)
        : simulator_(simulator),
          global_services_(global_services),
          environment_(environment),
          vehicles_(vehicles),
          service_packages_(service_packages),
          simulation_dt_(simulation_dt),
          add_error_(std::move(add_error)),
          add_warning_(std::move(add_warning)) {}

    void reset() {
        global_services_.clear();
        environment_ = EnvironmentInstance{};
        vehicles_.clear();
        selected_form_family_.clear();
        selected_form_family_by_scope_.clear();
        assembly_descriptors_.clear();
        service_finalization_tasks_.clear();
    }

    void installGlobalServices(const ConfigNode& global_service_config) {
        buildServices(global_service_config,
                      global_services_,
                      ServiceScopeInfo{ServiceScopeKind::Global,
                                       "global",
                                       "",
                                       "global_services"});
    }

    void buildMission(const ConfigNode& root) {
        buildEnvironment(root["environment"]);
        buildVehicles(root["vehicles"]);
        buildTermination(root["termination"]);
        buildSummary(root["summary"]);
    }

    void finalizeServices() {
        ServiceFinalizationContext context{
            simulator_.getRegistry(),
            [this](const std::string& message) { add_warning_(message); }};
        for (auto& task : service_finalization_tasks_) {
            try {
                if (task) {
                    task->finalize(context);
                }
            } catch (const std::exception& e) {
                add_error_(e.what());
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
        std::string scope_id;
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

    static bool isValidVehicleId(const std::string& id) {
        if (id.empty() || id == "env") {
            return false;
        }
        const auto valid_first = [](unsigned char ch) {
            return std::isalpha(ch) || ch == '_';
        };
        const auto valid_rest = [](unsigned char ch) {
            return std::isalnum(ch) || ch == '_' || ch == '-';
        };

        if (!valid_first(static_cast<unsigned char>(id.front()))) {
            return false;
        }
        return std::all_of(id.begin() + 1,
                           id.end(),
                           [&](char ch) {
                               return valid_rest(static_cast<unsigned char>(ch));
                           });
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

    std::string buildUnknownServiceMessage(const std::string& service_name) const {
        std::string message = "Unknown service '" + service_name + "'.";
        const auto available = service_packages_.listPackageIds();
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
                               const ConfigNode& config,
                               bool strict) const {
        auto unused = config.getUnusedKeys();
        if (unused.empty()) {
            return;
        }
        const std::string message =
            "Component '" + component_name + "' (type: " + type_name +
            ") has unrecognized config keys: " + joinStrings(unused) + ".";
        if (strict) {
            add_error_(message);
        } else {
            add_warning_(message);
        }
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
                      ServiceScopeInfo{ServiceScopeKind::Environment,
                                       "environment",
                                       "env",
                                       "environment.services"});

        PlacementSpec placement;
        placement.context = "environment.components";
        placement.placement = "environment";
        placement.name_prefix = "env.";
        placement.expected_role = ComponentPackageRole::Environment;
        placement.execution_stage = ExecutionStage::Environment;
        placement.owner_components = &environment_.components;
        registerComponents(environment_config["components"], placement);
    }

    void buildVehicles(const ConfigNode& vehicles_config) {
        if (!vehicles_config.isArray()) {
            add_error_("Top-level 'vehicles' must be an array.");
            return;
        }
        if (vehicles_config.size() == 0) {
            add_error_("Top-level 'vehicles' must contain at least one vehicle.");
            return;
        }

        std::unordered_set<std::string> vehicle_ids;
        for (size_t i = 0; i < vehicles_config.size(); ++i) {
            const auto& vehicle_config = vehicles_config[i];
            const std::string context = "vehicles[" + std::to_string(i) + "]";

            if (!vehicle_config.isObject()) {
                add_error_(context + " must be an object.");
                continue;
            }

            const std::string vehicle_id = vehicle_config["id"].asString();
            if (!isValidVehicleId(vehicle_id)) {
                add_error_(context +
                           " must define a valid 'id' matching [A-Za-z_][A-Za-z0-9_-]* "
                           "and not equal to 'env'.");
                continue;
            }
            if (!vehicle_ids.insert(vehicle_id).second) {
                add_error_("Duplicate vehicle id '" + vehicle_id + "'.");
                continue;
            }

            auto* vehicle = prepareVehicle(vehicle_id, vehicle_config, context);
            buildForm(*vehicle, vehicle_config["form"], context + ".form");
            buildVehicleGroups(*vehicle, vehicle_config, context);
            buildInteraction(*vehicle,
                             vehicle_config["interaction"],
                             context + ".interaction");
        }
    }

    VehicleInstance* prepareVehicle(const std::string& vehicle_id,
                                    const ConfigNode& vehicle_config,
                                    const std::string& config_path) {
        vehicles_.push_back(VehicleInstance{});
        auto& vehicle = vehicles_.back();
        vehicle.id = vehicle_id;

        if (vehicle_config.isNull()) {
            return &vehicle;
        }
        if (!vehicle_config.isObject()) {
            add_error_("Vehicle block '" + config_path + "' must be an object.");
            return &vehicle;
        }

        buildServices(vehicle_config["services"],
                      vehicle.services,
                      ServiceScopeInfo{ServiceScopeKind::Vehicle,
                                       vehicle_id,
                                       vehicle_id,
                                       config_path + ".services"});
        return &vehicle;
    }

    void buildForm(VehicleInstance& vehicle,
                   const ConfigNode& form_config,
                   const std::string& context) {
        if (form_config.isNull()) {
            add_error_("Mission configuration must define a '" + context + "' object.");
            return;
        }
        if (!form_config.isObject()) {
            add_error_("Block '" + context + "' must be an object.");
            return;
        }

        const std::string declared_family = form_config["family"].asString();
        if (!declared_family.empty()) {
            auto& selected_family = selected_form_family_by_scope_[vehicle.id];
            if (!selected_family.empty() && selected_family != declared_family) {
                add_error_("Form block '" + context +
                           "' declares form family '" + declared_family +
                           "' but the selected form family for vehicle '" +
                           vehicle.id + "' is '" + selected_family + "'.");
            } else {
                selected_family = declared_family;
            }
            if (selected_form_family_.empty()) {
                selected_form_family_ = declared_family;
            }
        }

        PlacementSpec placement;
        placement.context = context + ".components";
        placement.placement = "form";
        placement.name_prefix = vehicle.id + ".";
        placement.expected_role = ComponentPackageRole::Form;
        placement.execution_stage = ExecutionStage::Form;
        placement.scope_id = vehicle.id;
        placement.local_services = &vehicle.services;
        placement.owner_components = &vehicle.components;
        registerComponents(form_config["components"], placement);
    }

    void buildVehicleGroups(VehicleInstance& vehicle,
                            const ConfigNode& vehicle_config,
                            const std::string& context) {
        if (vehicle_config.isNull()) {
            return;
        }
        if (!vehicle_config.isObject()) {
            return;
        }

        auto* vehicle_services = &vehicle.services;
        auto* vehicle_components = &vehicle.components;
        const std::string vehicle_prefix = vehicle.id + ".";

        registerComponents(
            vehicle_config["common"],
            PlacementSpec{context + ".common",
                          "vehicle.common",
                          vehicle_prefix,
                          ComponentPackageRole::VehicleCommon,
                          ExecutionStage::None,
                          vehicle.id,
                          vehicle_services,
                          vehicle_components});
        registerComponents(
            vehicle_config["input"],
            PlacementSpec{context + ".input",
                          "vehicle.input",
                          vehicle_prefix,
                          ComponentPackageRole::VehicleInput,
                          ExecutionStage::VehicleInput,
                          vehicle.id,
                          vehicle_services,
                          vehicle_components});
        registerComponents(
            vehicle_config["process"],
            PlacementSpec{context + ".process",
                          "vehicle.process",
                          vehicle_prefix,
                          ComponentPackageRole::VehicleProcess,
                          ExecutionStage::VehicleProcess,
                          vehicle.id,
                          vehicle_services,
                          vehicle_components});
        registerComponents(
            vehicle_config["output"],
            PlacementSpec{context + ".output",
                          "vehicle.output",
                          vehicle_prefix,
                          ComponentPackageRole::VehicleOutput,
                          ExecutionStage::VehicleOutput,
                          vehicle.id,
                          vehicle_services,
                          vehicle_components});
    }

    void buildInteraction(VehicleInstance& vehicle,
                          const ConfigNode& interaction_config,
                          const std::string& context) {
        if (interaction_config.isNull()) {
            return;
        }
        if (!interaction_config.isObject()) {
            add_error_("Block '" + context + "' must be an object.");
            return;
        }

        auto* vehicle_services = &vehicle.services;
        auto* vehicle_components = &vehicle.components;

        registerComponents(
            interaction_config["components"],
            PlacementSpec{context + ".components",
                          "interaction",
                          vehicle.id + ".",
                          ComponentPackageRole::Interaction,
                          ExecutionStage::Interaction,
                          vehicle.id,
                          vehicle_services,
                          vehicle_components});
    }

    void buildTermination(const ConfigNode& termination_config) {
        if (termination_config.isNull()) {
            return;
        }
        if (!termination_config.isObject()) {
            add_error_("Top-level 'termination' must be an object.");
            return;
        }

        registerComponent(termination_config,
                          PlacementSpec{"termination",
                                        "termination",
                                        "mission.",
                                        ComponentPackageRole::Termination,
                                        ExecutionStage::Termination,
                                        "mission",
                                        nullptr,
                                        nullptr},
                          "termination");
    }

    void buildSummary(const ConfigNode& summary_config) {
        if (summary_config.isNull()) {
            return;
        }
        if (!summary_config.isObject()) {
            add_error_("Top-level 'summary' must be an object.");
            return;
        }

        registerComponent(summary_config,
                          PlacementSpec{"summary",
                                        "summary",
                                        "mission.",
                                        ComponentPackageRole::Summary,
                                        ExecutionStage::Summary,
                                        "mission",
                                        nullptr,
                                        nullptr},
                          "summary");
    }

    std::string selectedFormFamilyForScope(const std::string& scope_id) const {
        const auto it = selected_form_family_by_scope_.find(scope_id);
        if (it == selected_form_family_by_scope_.end()) {
            return "";
        }
        return it->second;
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
        if (placement.placement == "interaction" && type_form_family.empty()) {
            add_error_("Interaction component '" + full_name + "' of type '" +
                       type_name +
                       "' must declare a non-empty form family because interaction "
                       "is a form-specific closure that produces form input.");
        }

        if (placement.placement == "form") {
            if (type_form_family.empty()) {
                add_error_("Form component '" + full_name + "' of type '" +
                           type_name +
                           "' must declare a non-empty form family.");
                return;
            }
            if (!type_form_family.empty()) {
                auto& selected_family =
                    selected_form_family_by_scope_[placement.scope_id];
                if (selected_family.empty()) {
                    selected_family = type_form_family;
                } else if (selected_family != type_form_family) {
                    add_error_("Form component '" + full_name + "' of type '" + type_name +
                               "' advertises form family '" + type_form_family +
                               "' but the selected form family for vehicle '" +
                               placement.scope_id + "' is '" + selected_family + "'.");
                }
                if (selected_form_family_.empty()) {
                    selected_form_family_ = selected_family;
                }
            }
            return;
        }

        const std::string selected_family =
            selectedFormFamilyForScope(placement.scope_id);
        if (!selected_family.empty() && !type_form_family.empty() &&
            type_form_family != selected_family) {
            add_error_("Component '" + full_name + "' of type '" + type_name +
                       "' targets form family '" + type_form_family +
                       "' but the selected form family for vehicle '" +
                       placement.scope_id + "' is '" + selected_family + "'.");
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

        for (size_t i = 0; i < components.size(); ++i) {
            const auto& component_config = components[i];
            const std::string component_path =
                placement.context + "[" + std::to_string(i) + "]";
            registerComponent(component_config, placement, component_path);
        }
    }

    bool configureScheduling(ComponentBase* component,
                             const ConfigNode& component_config,
                             const std::string& component_path) {
        int priority = 0;
        const auto& priority_node = component_config["priority"];
        if (!priority_node.isNull()) {
            if (!priority_node.isNumber()) {
                add_error_(component_path + ".priority must be an integer number.");
                return false;
            }
            const double raw_priority = priority_node.asDouble();
            if (!std::isfinite(raw_priority) ||
                std::floor(raw_priority) != raw_priority ||
                raw_priority < static_cast<double>(std::numeric_limits<int>::min()) ||
                raw_priority > static_cast<double>(std::numeric_limits<int>::max())) {
                add_error_(component_path + ".priority must be an integer number.");
                return false;
            }
            priority = static_cast<int>(raw_priority);
        }
        component->setPriorityInternal_(priority);

        const auto& rate_node = component_config["rate_hz"];
        if (rate_node.isNull()) {
            return true;
        }
        if (!rate_node.isNumber()) {
            add_error_(component_path + ".rate_hz must be a positive number.");
            return false;
        }
        const double rate_hz = rate_node.asDouble();
        if (!validateRateHz(component_path, rate_hz)) {
            return false;
        }
        component->setExecutionFrequency(rate_hz);
        return true;
    }

    bool validateRateHz(const std::string& component_path, double rate_hz) {
        if (!std::isfinite(rate_hz) || rate_hz <= 0.0) {
            add_error_(component_path + ".rate_hz must be > 0.");
            return false;
        }
        if (simulation_dt_ <= 0.0) {
            add_error_(component_path +
                       ".rate_hz cannot be validated because simulation.dt must be > 0.");
            return false;
        }

        const double simulation_frequency_hz = 1.0 / simulation_dt_;
        const double tolerance =
            1.0e-9 * std::max(1.0, std::abs(simulation_frequency_hz));
        if (rate_hz > simulation_frequency_hz + tolerance) {
            add_error_(component_path +
                       ".rate_hz must not exceed the simulation frequency 1 / simulation.dt.");
            return false;
        }

        const double raw_interval = simulation_frequency_hz / rate_hz;
        const double rounded_interval = std::round(raw_interval);
        const double interval_tolerance =
            1.0e-9 * std::max(1.0, std::abs(raw_interval));
        if (std::abs(raw_interval - rounded_interval) > interval_tolerance ||
            rounded_interval < 1.0) {
            add_error_(component_path +
                       ".rate_hz must divide the simulation frequency into an integer step interval.");
            return false;
        }
        return true;
    }

    bool validateRequiredPlacementInterface(ComponentBase* component,
                                            const std::string& type_name,
                                            const std::string& full_name,
                                            const PlacementSpec& placement) {
        if (placement.placement == "termination" &&
            !dynamic_cast<interfaces::ITerminationEvaluator*>(component)) {
            add_error_("Termination component '" + full_name + "' of type '" +
                       type_name + "' must implement ITerminationEvaluator.");
            return false;
        }
        if (placement.placement == "summary" &&
            !dynamic_cast<interfaces::ISummaryObserver*>(component)) {
            add_error_("Summary component '" + full_name + "' of type '" +
                       type_name + "' must implement ISummaryObserver.");
            return false;
        }
        return true;
    }

    void registerComponent(const ConfigNode& component_config,
                           const PlacementSpec& placement,
                           const std::string& component_path) {
        const std::string type_name = component_config["type"].asString();
        const std::string base_name = component_config["name"].asString();
        const std::string full_name = placement.name_prefix + base_name;
        const std::string config_path = component_path + ".config";

        auto& registry = simulator_.getRegistry();
        auto& factory = ComponentFactory::instance();

        if (type_name.empty() || base_name.empty()) {
            add_error_("Component at " + component_path + " is missing type or name.");
            return;
        }
        if (!factory.hasType(type_name)) {
            add_error_(buildUnknownTypeMessage(type_name,
                                               full_name,
                                               factory,
                                               placement.context));
            return;
        }
        if (registry.has(full_name)) {
            add_error_("Duplicate component name '" + full_name + "'.");
            return;
        }

        validatePlacement(type_name, full_name, placement, factory);

        auto component = factory.create(type_name);
        auto* component_ptr = component.get();
        annotateComponentMetadata(component_ptr, type_name, factory);

        if (!configureScheduling(component_ptr, component_config, component_path) ||
            !validateRequiredPlacementInterface(component_ptr,
                                                type_name,
                                                full_name,
                                                placement)) {
            return;
        }

        const auto& config = component_config["config"];
        config.resetAccessTracking();
        try {
            component_ptr->configure(config, config_path);
        } catch (const std::exception& e) {
            add_error_("Component '" + full_name + "' of type '" + type_name +
                       "' failed to configure at " + config_path + ": " +
                       e.what());
            return;
        }
        checkUnusedConfigKeys(full_name,
                              type_name,
                              config,
                              factory.getCategory(type_name) ==
                                  ComponentCategory::Builtin);

        try {
            component_ptr->injectServices(global_services_);
            component_ptr->injectServices(environment_.services);
            if (placement.local_services) {
                component_ptr->injectServices(*placement.local_services);
            }
        } catch (const std::exception& e) {
            add_error_("Component '" + full_name + "' of type '" + type_name +
                       "' failed service injection: " + e.what());
            return;
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
            factory.getFormFamily(type_name),
            placement.scope_id,
            selectedFormFamilyForScope(placement.scope_id)});

        registry.addDynamic(full_name,
                            std::move(component),
                            factory.getInterfaces(type_name));
    }

    void buildServices(const ConfigNode& service_config,
                       ServiceContext& services,
                       ServiceScopeInfo scope) {
        if (service_config.isNull()) {
            return;
        }
        if (!service_config.isObject()) {
            add_error_("Service block for scope '" + scope.name +
                       "' must be an object.");
            return;
        }

        for (const auto& [service_name, config] : service_config) {
            if (!config.isObject()) {
                add_error_("Service '" + service_name + "' in scope '" +
                           scope.name + "' must be an object.");
                continue;
            }
            if (config.has("enabled") && !config["enabled"].asBool(true)) {
                continue;
            }

            const auto* package = service_packages_.findPackage(service_name);
            if (!package) {
                add_error_(buildUnknownServiceMessage(service_name));
                continue;
            }
            if (!package->supportsScope(scope.kind)) {
                add_error_("Service '" + service_name + "' is only supported in scope " +
                           joinServiceScopeNames(package->supportedScopes()) + ".");
                continue;
            }

            try {
                ServiceCreationContext context{services, config, scope};
                auto task = package->create(context);
                if (task) {
                    service_finalization_tasks_.push_back(std::move(task));
                }
            } catch (const std::exception& e) {
                add_error_("Service '" + service_name + "' in scope '" +
                           scope.name + "' failed to create: " + e.what());
            }
        }
    }

    Simulator& simulator_;
    ServiceContext& global_services_;
    EnvironmentInstance& environment_;
    std::vector<VehicleInstance>& vehicles_;
    ServicePackageRegistry& service_packages_;
    double simulation_dt_ = 0.0;
    DiagnosticReporter add_error_;
    DiagnosticReporter add_warning_;
    std::string selected_form_family_;
    std::unordered_map<std::string, std::string> selected_form_family_by_scope_;
    std::vector<AssemblyDescriptor> assembly_descriptors_;
    std::vector<std::unique_ptr<IServiceFinalizationTask>> service_finalization_tasks_;
};

} // namespace gnc::core
