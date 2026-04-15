#pragma once

#include "gnc/core/config_manager.hpp"
#include "gnc/core/integrators/euler_integrator.hpp"
#include "gnc/core/integrators/rk4_integrator.hpp"
#include "gnc/core/mission_assembler.hpp"
#include "gnc/core/simulator.hpp"
#include "gnc/infrastructure/dependency_validator.hpp"
#include "gnc/common/string_utils.hpp"
#include "gnc/interfaces/i_continuous_system.hpp"
#include "gnc/interfaces/i_observable.hpp"

#include <algorithm>
#include <sstream>
#include <unordered_set>

namespace gnc::core {

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

        MissionAssembler assembler(
            simulator_,
            global_services_,
            environment_,
            vehicles_,
            deferred_service_actions_,
            [this](const std::string& message) { addBuildError(message); },
            [this](const std::string& message) { addBuildWarning(message); });
        assembler.reset();

        const auto& simulation = config_.simulation();
        SimulatorConfig simulator_config;
        simulator_config.dt = simulation["dt"].asDouble(0.01);
        simulator_config.duration = simulation["duration"].asDouble(10.0);
        simulator_.configure(simulator_config);
        buildIntegrator();

        assembler.installGlobalServices(config_.globalServices());
        const auto& entities = config_.entities();
        if (!entities.isArray()) {
            addBuildError("Mission configuration must define an 'entities' array.");
        } else {
            assembler.buildEntities(entities);
        }

        assembler.runDeferredServiceActions();

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


    ConfigManager config_;
    Simulator simulator_;
    ServiceContext global_services_;
    EnvironmentInstance environment_;
    std::vector<VehicleInstance> vehicles_;
    std::vector<PluginRegistry::DeferredAction> deferred_service_actions_;
    std::vector<std::string> build_errors_;
    std::vector<std::string> build_warnings_;
};

} // namespace gnc::core
