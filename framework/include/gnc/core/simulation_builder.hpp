#pragma once

#include "gnc/core/component_factory.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/core/integrators/euler_integrator.hpp"
#include "gnc/core/integrators/rk4_integrator.hpp"
#include "gnc/core/plugin_registry.hpp"
#include "gnc/core/service_context.hpp"
#include "gnc/core/simulator.hpp"
#include "gnc/infrastructure/dependency_validator.hpp"
#include "gnc/common/string_utils.hpp"
#include "gnc/interfaces/i_continuous_system.hpp"
#include "gnc/interfaces/i_observable.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <unordered_set>

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

class SimulationBuilder {
public:
    bool loadConfig(const std::string& filename) {
        return config_.loadFromFile(filename);
    }

    bool loadConfigString(const std::string& json) {
        return config_.loadFromString(json);
    }

    Simulator& build() {
        build_errors_.clear();
        build_warnings_.clear();
        deferred_service_actions_.clear();

        const auto& simulation = config_.simulation();
        SimulatorConfig simulator_config;
        simulator_config.dt = simulation["dt"].asDouble(0.01);
        simulator_config.duration = simulation["duration"].asDouble(10.0);
        simulator_.configure(simulator_config);
        buildIntegrator();

        buildServices(config_.globalServices(), global_services_, "global", "");
        buildEnvironment();

        if (config_.isMultiVehicle()) {
            buildMultiVehicle();
        } else {
            buildSingleVehicle();
        }

        runDeferredServiceActions();

        auto validation = DependencyValidator::validate(simulator_.getRegistry());
        for (const auto& error : validation.errors) {
            addBuildError(error);
        }
        for (const auto& warning : validation.warnings) {
            addBuildWarning(warning);
        }

        preflightDependencyBindings(validation.failed_components);
        buildStopConditions();

        if (!simulator_.initializeAutoDataLogger(config_.root()["outputs"])) {
            addBuildError("AutoDataLogger initialization failed during simulation build.");
        }

        if (!reportBuildDiagnostics()) {
            throw std::runtime_error("Simulation build failed with " +
                                     std::to_string(build_errors_.size()) +
                                     " error(s).");
        }

        logComponentInventory();
        return simulator_;
    }

    ConfigManager& getConfigManager() { return config_; }
    Simulator& getSimulator() { return simulator_; }
    ServiceContext& getGlobalServices() { return global_services_; }
    EnvironmentInstance& getEnvironment() { return environment_; }
    std::vector<VehicleInstance>& getVehicles() { return vehicles_; }

private:
    static std::string extractScope(const std::string& full_name) {
        const auto pos = full_name.find('.');
        if (pos == std::string::npos) {
            return "";
        }
        return full_name.substr(0, pos + 1);
    }

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

    static std::string joinDiagnosticLines(const std::vector<std::string>& values) {
        std::string result;
        for (size_t i = 0; i < values.size(); ++i) {
            result += (i == 0 ? "  - " : "\n  - ");
            result += values[i];
        }
        return result.empty() ? "  - (none)" : result;
    }

    static std::string listFieldNames(
        const std::vector<interfaces::ObservableField>& fields) {
        std::vector<std::string> names;
        names.reserve(fields.size());
        for (const auto& field : fields) {
            names.push_back(field.name);
        }
        return joinStrings(names);
    }

    static std::string listStateFieldNames(
        const interfaces::IContinuousSystem* continuous_system) {
        if (!continuous_system) {
            return "(none)";
        }
        return joinStrings(continuous_system->getStateLayout().names());
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
        std::string message = "Unknown service plugin '" + service_name + "'.";
        const auto available = PluginRegistry::instance().getServiceNames();
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

    void annotateComponentMetadata(ComponentBase* component,
                                   const std::string& type_name,
                                   const ComponentFactory& factory) {
        if (!component) {
            return;
        }
        component->setTypeNameInternal_(type_name);
        component->setComponentCategoryInternal_(toString(factory.getCategory(type_name)));
        component->setRegistrationOriginInternal_(factory.getRegistrationOrigin(type_name));
    }

    void logComponentInventory() {
        std::vector<std::string> builtin_types;
        std::vector<std::string> project_types;

        for (const auto& name : simulator_.getRegistry().getComponentNames()) {
            auto* component = simulator_.getRegistry().get<ComponentBase>(name);
            if (!component) {
                continue;
            }

            if (component->getComponentCategory() == "builtin") {
                builtin_types.push_back(component->getTypeName());
            } else {
                project_types.push_back(component->getTypeName());
            }
        }

        auto unique_join = [](std::vector<std::string> values) {
            std::sort(values.begin(), values.end());
            values.erase(std::unique(values.begin(), values.end()), values.end());
            return joinStrings(values);
        };

        LOG_INFO("Mission component inventory: builtin types [{}], project types [{}]",
                 unique_join(std::move(builtin_types)),
                 unique_join(std::move(project_types)));
    }

    void preflightDependencyBindings(
        const std::unordered_set<std::string>& validation_failed_components) {
        auto& registry = simulator_.getRegistry();
        for (auto* component : registry.getAllComponents()) {
            if (validation_failed_components.count(component->getName()) > 0) {
                continue;
            }

            ScopedRegistry::BindingDiagnostics diagnostics;
            try {
                ScopedRegistry scoped(extractScope(component->getName()),
                                      registry,
                                      component->getName(),
                                      &diagnostics);
                component->injectDependencies(scoped);
            } catch (const std::exception& e) {
                addBuildError("Component '" + component->getName() +
                              "' failed dependency preflight: " + e.what());
                continue;
            }

            if (!diagnostics.errors.empty()) {
                addBuildError("Component '" + component->getName() +
                              "' failed dependency preflight with " +
                              std::to_string(diagnostics.errors.size()) +
                              " required binding issue(s):\n" +
                              joinDiagnosticLines(diagnostics.errors));
                continue;
            }

            for (const auto& warning : diagnostics.warnings) {
                addBuildWarning("Component '" + component->getName() +
                                "' optional dependency warning: " + warning);
            }

            component->markDependenciesInjectedInternal_();
        }
    }

    void buildIntegrator() {
        const std::string integrator_name =
            config_.simulation()["integrator"].asString("rk4");
        if (integrator_name == "rk4") {
            simulator_.setIntegrator(std::make_unique<RK4Integrator>());
            return;
        }
        if (integrator_name == "euler") {
            simulator_.setIntegrator(std::make_unique<EulerIntegrator>());
            return;
        }

        addBuildWarning("Unknown integrator '" + integrator_name +
                        "'. Falling back to RK4.");
        simulator_.setIntegrator(std::make_unique<RK4Integrator>());
    }

    void addBuildError(const std::string& message) {
        build_errors_.push_back(message);
    }

    void addBuildWarning(const std::string& message) {
        build_warnings_.push_back(message);
    }

    void checkUnusedConfigKeys(const std::string& component_name,
                               const std::string& type_name,
                               const ConfigNode& config) {
        auto unused = config.getUnusedKeys();
        if (unused.empty()) {
            return;
        }
        addBuildWarning("Component '" + component_name + "' (type: " + type_name +
                        ") has unrecognized config keys: " + joinStrings(unused) + ".");
    }

    bool reportBuildDiagnostics() {
        if (build_errors_.empty() && build_warnings_.empty()) {
            return true;
        }

        std::ostringstream report;
        report << "\n=== Simulation Build Diagnostics ===\n";

        if (!build_errors_.empty()) {
            report << "[ERRORS] (" << build_errors_.size() << ")\n";
            for (size_t i = 0; i < build_errors_.size(); ++i) {
                report << "  " << (i + 1) << ". " << build_errors_[i] << "\n";
            }
        }

        if (!build_warnings_.empty()) {
            report << "[WARNINGS] (" << build_warnings_.size() << ")\n";
            for (size_t i = 0; i < build_warnings_.size(); ++i) {
                report << "  " << (i + 1) << ". " << build_warnings_[i] << "\n";
            }
        }

        report << "====================================";
        if (!build_errors_.empty()) {
            LOG_ERROR("{}", report.str());
            return false;
        }

        LOG_WARNING("{}", report.str());
        return true;
    }

    bool buildStopConditions() {
        const auto& conditions = config_.simulation()["stop_conditions"];
        if (conditions.isNull()) {
            return true;
        }
        if (!conditions.isArray()) {
            addBuildWarning("simulation.stop_conditions must be an array.");
            return false;
        }

        auto& registry = simulator_.getRegistry();
        bool success = true;

        for (size_t i = 0; i < conditions.size(); ++i) {
            const auto& condition = conditions[i];
            const std::string type = condition["type"].asString();
            const std::string component_name = condition["component"].asString();
            const std::string field_name = condition["field"].asString();
            const double threshold = condition["value"].asDouble(0.0);
            const std::string description = condition["description"].asString(
                type + "(" + component_name + "." + field_name + ", " +
                std::to_string(threshold) + ")");

            if (type.empty() || component_name.empty() || field_name.empty()) {
                addBuildWarning("Stop condition at index " + std::to_string(i) +
                                " is missing required fields.");
                success = false;
                continue;
            }

            auto* component = registry.get<ComponentBase>(component_name);
            if (!component) {
                std::string message = "Stop condition references unknown component '" +
                                      component_name + "'.";
                const std::string suggestion =
                    common::findClosestMatch(component_name, registry.getComponentNames());
                if (!suggestion.empty()) {
                    message += " Did you mean '" + suggestion + "'?";
                }
                addBuildWarning(message);
                success = false;
                continue;
            }

            auto* observable = dynamic_cast<interfaces::IObservable*>(component);
            auto* continuous_system =
                dynamic_cast<interfaces::IContinuousSystem*>(component);

            std::function<double()> getter;
            std::vector<std::string> candidate_fields;

            if (observable) {
                const auto fields = observable->getObservableFields();
                candidate_fields.reserve(fields.size());
                for (const auto& field : fields) {
                    candidate_fields.push_back(field.name);
                    if (!getter && field.name == field_name) {
                        getter = field.getter;
                    }
                }
            }

            if (!getter && continuous_system &&
                continuous_system->getStateLayout().has(field_name)) {
                getter = [continuous_system, field_name]() {
                    return continuous_system->getStateValue(field_name);
                };
            }

            if (!getter && continuous_system) {
                const auto& state_names = continuous_system->getStateLayout().names();
                candidate_fields.insert(candidate_fields.end(),
                                        state_names.begin(),
                                        state_names.end());
            }

            if (!getter) {
                std::sort(candidate_fields.begin(), candidate_fields.end());
                candidate_fields.erase(std::unique(candidate_fields.begin(),
                                                  candidate_fields.end()),
                                       candidate_fields.end());

                std::string message = "Stop condition references field '" + field_name +
                                      "' not found in component '" +
                                      component_name + "'.";
                if (observable) {
                    message += " Available observable fields: " +
                               listFieldNames(observable->getObservableFields()) + ".";
                }
                if (continuous_system) {
                    message += " Available state fields: " +
                               listStateFieldNames(continuous_system) + ".";
                }
                const std::string suggestion =
                    common::findClosestMatch(field_name, candidate_fields);
                if (!suggestion.empty()) {
                    message += " Did you mean '" + suggestion + "'?";
                }
                addBuildWarning(message);
                success = false;
                continue;
            }

            if (type == "component_field_below") {
                simulator_.addTerminationCondition(
                    description,
                    [getter, threshold](int, double) { return getter() < threshold; });
            } else if (type == "component_field_above") {
                simulator_.addTerminationCondition(
                    description,
                    [getter, threshold](int, double) { return getter() > threshold; });
            } else {
                addBuildWarning("Unknown stop condition type '" + type + "'.");
                success = false;
            }
        }

        return success;
    }

    void buildEnvironment() {
        const auto& environment_config = config_.root()["environment"];
        if (environment_config.isNull()) {
            return;
        }

        environment_.id = "environment";
        buildServices(environment_config["services"], environment_.services, "environment", "env");
        registerComponents(environment_config["components"],
                           "environment configuration",
                           "env.",
                           global_services_,
                           environment_.services,
                           nullptr,
                           environment_.components);
    }

    void buildSingleVehicle() {
        buildServices(config_.services(), global_services_, "flight_vehicle", "");
        registerComponents(config_.components(),
                           "single-vehicle configuration",
                           "",
                           global_services_,
                           environment_.services,
                           nullptr,
                           scratch_components_);
    }

    void buildMultiVehicle() {
        const auto& vehicle_configs = config_.vehicles();
        vehicles_.reserve(vehicle_configs.size());
        for (size_t i = 0; i < vehicle_configs.size(); ++i) {
            const auto& vehicle_config = vehicle_configs[i];
            VehicleInstance vehicle;
            vehicle.id = vehicle_config["id"].asString();
            if (vehicle.id.empty()) {
                addBuildError("Vehicle at index " + std::to_string(i) +
                              " is missing an id.");
                continue;
            }

            buildServices(vehicle_config["services"],
                          vehicle.services,
                          vehicle.id,
                          vehicle.id);
            registerComponents(vehicle_config["components"],
                               "vehicle '" + vehicle.id + "'",
                               vehicle.id + ".",
                               global_services_,
                               environment_.services,
                               &vehicle.services,
                               vehicle.components);
            vehicles_.push_back(std::move(vehicle));
        }
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
                addBuildError("Component at index " + std::to_string(i) +
                              " in " + context + " is missing type or name.");
                continue;
            }
            if (!factory.hasType(type_name)) {
                addBuildError(buildUnknownTypeMessage(type_name, full_name, factory, context));
                continue;
            }
            if (registry.has(full_name)) {
                addBuildError("Duplicate component name '" + full_name + "'.");
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
            addBuildError("Service block for scope '" + service_scope_name +
                          "' must be an object.");
            return;
        }

        for (const auto& [service_name, config] : service_config) {
            if (!config.isObject()) {
                addBuildError("Service '" + service_name + "' in scope '" +
                              service_scope_name + "' must be an object.");
                continue;
            }
            if (config.has("enabled") && !config["enabled"].asBool(true)) {
                continue;
            }
            if (!PluginRegistry::instance().hasServiceInstaller(service_name)) {
                addBuildError(buildUnknownServiceMessage(service_name));
                continue;
            }

            try {
                PluginRegistry::ServiceInstallRequest request{
                    config,
                    services,
                    service_scope_name,
                    registry_scope,
                    deferred_service_actions_};
                PluginRegistry::instance().installService(service_name, request);
            } catch (const std::exception& e) {
                addBuildError("Service '" + service_name + "' in scope '" +
                              service_scope_name + "' failed to install: " + e.what());
            }
        }
    }

    void runDeferredServiceActions() {
        auto& registry = simulator_.getRegistry();
        for (const auto& action : deferred_service_actions_) {
            try {
                action(registry);
            } catch (const std::exception& e) {
                addBuildError("Deferred service installation failed: " +
                              std::string(e.what()));
            }
        }
    }

    ConfigManager config_;
    Simulator simulator_;
    ServiceContext global_services_;
    EnvironmentInstance environment_;
    std::vector<VehicleInstance> vehicles_;
    std::vector<ComponentBase*> scratch_components_;
    std::vector<PluginRegistry::DeferredAction> deferred_service_actions_;
    std::vector<std::string> build_errors_;
    std::vector<std::string> build_warnings_;
};

} // namespace gnc::core
