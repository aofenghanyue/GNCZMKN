/**
 * @file simulation_builder.hpp
 * @brief 仿真构建器
 * 
 * 整合 Factory + Config，从配置文件构建完整仿真
 * 支持显式服务配置、多飞行器和Environment实体
 */
#pragma once

#include "simulator.hpp"
#include "component_factory.hpp"
#include "config_manager.hpp"
#include "dependency_validator.hpp"
#include "integrators/euler_integrator.hpp"
#include "integrators/rk4_integrator.hpp"
#include "service_context.hpp"
#include "string_utils.hpp"
#include "gnc/common/logger.hpp"
#include "gnc/services/coordinate/coordinate_service.hpp"
#include "gnc/services/coordinate/soviet_coordinate_system.hpp"
#include <algorithm>
#include <cctype>
#include <sstream>
#include <unordered_set>

namespace gnc::core {

/**
 * @brief 实体基类
 * 
 * Vehicle和Environment的公共基类
 */
struct EntityInstance {
    std::string id;
    ServiceContext services;
    std::vector<ComponentBase*> components;
};

/**
 * @brief 飞行器实例
 */
struct VehicleInstance : EntityInstance {
    // 可扩展飞行器特有属性
};

/**
 * @brief 环境实例
 * 
 * 全局上下文实体，用于：
 * - 时间推进
 * - 星历计算
 * - 碰撞检测
 * - 真值记录
 * 
 * 特点：
 * - 与Vehicle同级，但不是飞行器
 * - 先于所有Vehicle组件执行
 * - 可访问全局服务和所有Vehicle状态
 */
struct EnvironmentInstance : EntityInstance {
    // 环境特有属性
};

/**
 * @brief 仿真构建器
 * 
 * 从 JSON 配置文件构建完整的仿真环境
 * 
 * 架构层级：
 * - Simulation
 *   ├── Environment (全局上下文实体)
 *   └── Vehicles[] (飞行器实体)
 * 
 * 配置示例：
 * @code
 * {
 *   "simulation": { "dt": 0.01 },
 *   "environment": {
 *     "services": { "ephemeris": { "enabled": true } },
 *     "components": [
 *       { "type": "TimeComponent", "name": "time" }
 *     ]
 *   },
 *   "vehicles": [
 *     { "id": "chaser", "services": {...}, "components": [...] },
 *     { "id": "target", "services": {...}, "components": [...] }
 *   ]
 * }
 * @endcode
 */
class SimulationBuilder {
public:
    SimulationBuilder() = default;
    
    /// 加载配置文件
    bool loadConfig(const std::string& filename) {
        return config_.loadFromFile(filename);
    }
    
    /// 加载配置字符串
    bool loadConfigString(const std::string& json) {
        return config_.loadFromString(json);
    }
    
    /// 构建仿真器
    Simulator& build() {
        build_errors_.clear();
        build_warnings_.clear();
        pending_coordinate_installations_.clear();

        const auto& sim_config = config_.simulation();
        SimulatorConfig cfg;
        cfg.dt = sim_config["dt"].asDouble(0.01);
        cfg.duration = sim_config["duration"].asDouble(10.0);
        simulator_.configure(cfg);
        buildIntegrator();
        
        LOG_INFO("Simulation config: dt={}, duration={}", cfg.dt, cfg.duration);
        
        buildServices(config_.globalServices(), globalServices_, "global", "");
        
        buildEnvironment();
        
        if (config_.isMultiVehicle()) {
            buildMultiVehicle();
        } else {
            buildSingleVehicle();
        }

        bindCoordinateServices();
        
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
            addBuildError("AutoDataLogger initialization failed during simulation build. Please check the outputs configuration and output directory settings.");
        }

        if (!reportBuildDiagnostics()) {
            throw std::runtime_error("Simulation build failed with " +
                                     std::to_string(build_errors_.size()) +
                                     " error(s). See diagnostics report above.");
        }

        logComponentInventory();
        
        return simulator_;
    }
    
    /// 获取配置管理器
    ConfigManager& getConfigManager() { return config_; }
    
    /// 获取仿真器
    Simulator& getSimulator() { return simulator_; }
    
    /// 获取全局服务上下文
    ServiceContext& getGlobalServices() { return globalServices_; }
    
    /// 获取环境实例
    EnvironmentInstance& getEnvironment() { return environment_; }
    
    /// 获取飞行器实例（多飞行器模式）
    std::vector<VehicleInstance>& getVehicles() { return vehicles_; }
    
    /// 按ID获取飞行器
    VehicleInstance* getVehicle(const std::string& id) {
        for (auto& v : vehicles_) {
            if (v.id == id) return &v;
        }
        return nullptr;
    }

private:
    static std::string extractScope(const std::string& full_name) {
        const auto pos = full_name.find('.');
        if (pos == std::string::npos) {
            return "";
        }
        return full_name.substr(0, pos + 1);
    }

    static std::string normalizeCoordinateSchemeName(std::string scheme) {
        for (char& ch : scheme) {
            ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        }
        return scheme;
    }

    static std::string joinStrings(const std::vector<std::string>& values) {
        std::string result;
        for (size_t i = 0; i < values.size(); ++i) {
            if (i > 0) result += ", ";
            result += values[i];
        }
        return result.empty() ? "(none)" : result;
    }

    static std::string joinDiagnosticLines(const std::vector<std::string>& values) {
        std::string result;
        for (size_t i = 0; i < values.size(); ++i) {
            if (i > 0) result += "\n  - ";
            else result += "  - ";
            result += values[i];
        }
        return result.empty() ? "  - (none)" : result;
    }

    static std::string listFieldNames(const std::vector<interfaces::ObservableField>& fields) {
        std::vector<std::string> names;
        names.reserve(fields.size());
        for (const auto& field : fields) {
            names.push_back(field.name);
        }
        return joinStrings(names);
    }

    static std::string listStateFieldNames(const interfaces::IDynamicsModel* dynamics) {
        if (!dynamics) {
            return "(none)";
        }
        return joinStrings(dynamics->getStateLayout().names());
    }

    static std::string buildUnknownTypeMessage(const std::string& type_name,
                                               const std::string& component_name,
                                               const ComponentFactory& factory,
                                               const std::string& context = "") {
        std::string message = "Unknown component type: '" + type_name + "' (component name: '" + component_name + "')";
        if (!context.empty()) {
            message += " in " + context;
        }
        message += ". Available registered types: " + factory.describeRegisteredTypes();

        const std::string suggestion = findClosestMatch(type_name, factory.getRegisteredTypes());
        if (!suggestion.empty()) {
            message += ". Did you mean '" + suggestion + "'?";
        }
        message += ". Please register the component or fix the type name.";
        message += " User-project components are auto-discovered from the active user/<project>/components root; use --list-components or --list-components-verbose to inspect the currently registered starter/custom types.";
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
        std::vector<std::string> starter_types;
        std::vector<std::string> custom_types;

        for (const auto& name : simulator_.getRegistry().getComponentNames()) {
            auto* component = simulator_.getRegistry().get<ComponentBase>(name);
            if (!component) {
                continue;
            }

            if (component->getComponentCategory() == "starter") {
                starter_types.push_back(component->getTypeName());
            } else {
                custom_types.push_back(component->getTypeName());
            }
        }

        auto unique_join = [](std::vector<std::string> values) {
            std::sort(values.begin(), values.end());
            values.erase(std::unique(values.begin(), values.end()), values.end());
            return joinStrings(values);
        };

        LOG_INFO("Mission component inventory: starter types [{}], custom/example types [{}]",
                 unique_join(std::move(starter_types)),
                 unique_join(std::move(custom_types)));
    }

    void preflightDependencyBindings(const std::unordered_set<std::string>& validation_failed_components) {
        auto& registry = simulator_.getRegistry();
        for (auto* component : registry.getAllComponents()) {
            if (validation_failed_components.find(component->getName()) != validation_failed_components.end()) {
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
                              "' failed implicit dependency injection preflight: " +
                              e.what());
                continue;
            }

            if (!diagnostics.errors.empty()) {
                addBuildError("Component '" + component->getName() +
                              "' failed implicit dependency injection preflight with " +
                              std::to_string(diagnostics.errors.size()) +
                              " required binding issue(s):\n" +
                              joinDiagnosticLines(diagnostics.errors));
                continue;
            }

            for (const auto& warning : diagnostics.warnings) {
                addBuildWarning("Component '" + component->getName() +
                                "' optional dependency preflight warning: " +
                                warning);
            }

            component->markDependenciesInjectedInternal_();
        }
    }

    void buildIntegrator() {
        const std::string name = config_.simulation()["integrator"].asString("rk4");
        if (name == "rk4") {
            simulator_.setIntegrator(std::make_unique<RK4Integrator>());
            return;
        }
        if (name == "euler") {
            simulator_.setIntegrator(std::make_unique<EulerIntegrator>());
            return;
        }

        addBuildWarning("Unknown integrator '" + name + "'. Using RK4.");
        simulator_.setIntegrator(std::make_unique<RK4Integrator>());
    }

    void addBuildError(const std::string& msg) {
        build_errors_.push_back(msg);
    }

    void addBuildWarning(const std::string& msg) {
        build_warnings_.push_back(msg);
    }

    void checkUnusedConfigKeys(const std::string& component_name,
                               const std::string& type_name,
                               const ConfigNode& config) {
        auto unused = config.getUnusedKeys();
        if (unused.empty()) {
            return;
        }

        std::string unused_str;
        for (size_t i = 0; i < unused.size(); ++i) {
            if (i > 0) unused_str += ", ";
            unused_str += "'" + unused[i] + "'";
        }

        addBuildWarning("Component '" + component_name + "' (type: " + type_name +
                        "): unrecognized config key(s): " + unused_str +
                        ". These may be typos. Please check your configuration.");
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
            addBuildWarning("Simulation stop_conditions must be an array. The configured stop conditions will be ignored.");
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
                type + "(" + component_name + "." + field_name + ", " + std::to_string(threshold) + ")");

            if (type.empty() || component_name.empty() || field_name.empty()) {
                addBuildWarning("Stop condition at index " + std::to_string(i) + " is missing required fields (type, component, field). Please provide all required fields.");
                success = false;
                continue;
            }

            auto* component = registry.get<ComponentBase>(component_name);
            if (!component) {
                std::string message = "Stop condition references unknown component '" + component_name + "'.";
                const std::string suggestion = findClosestMatch(component_name, registry.getComponentNames());
                if (!suggestion.empty()) {
                    message += " Did you mean '" + suggestion + "'?";
                }
                message += " Please fix the component name in simulation.stop_conditions.";
                addBuildWarning(message);
                success = false;
                continue;
            }

            auto* observable = dynamic_cast<interfaces::IObservable*>(component);
            auto* dynamics = dynamic_cast<interfaces::IDynamicsModel*>(component);

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

            if (!getter && dynamics && dynamics->getStateLayout().has(field_name)) {
                getter = [dynamics, field_name]() {
                    return dynamics->getStateValue(field_name);
                };
            }

            if (!getter && dynamics) {
                const auto& state_names = dynamics->getStateLayout().names();
                candidate_fields.insert(candidate_fields.end(), state_names.begin(), state_names.end());
            }

            if (!getter) {
                std::sort(candidate_fields.begin(), candidate_fields.end());
                candidate_fields.erase(std::unique(candidate_fields.begin(), candidate_fields.end()), candidate_fields.end());

                std::string message = "Stop condition references field '" + field_name + "' not found in component '" + component_name + "'.";
                if (observable) {
                    message += " Available observable fields: " + listFieldNames(observable->getObservableFields()) + ".";
                }
                if (dynamics) {
                    message += " Available dynamics state fields: " + listStateFieldNames(dynamics) + ".";
                }
                if (!observable && !dynamics) {
                    message += " The component neither implements IObservable nor IDynamicsModel.";
                }

                const std::string suggestion = findClosestMatch(field_name, candidate_fields);
                if (!suggestion.empty()) {
                    message += " Did you mean '" + suggestion + "'?";
                }
                message += " Please fix the field name in simulation.stop_conditions.";
                addBuildWarning(message);
                success = false;
                continue;
            }

            if (type == "component_field_below") {
                simulator_.addTerminationCondition(description, [getter, threshold](int, double) {
                    return getter() < threshold;
                });
            } else if (type == "component_field_above") {
                simulator_.addTerminationCondition(description, [getter, threshold](int, double) {
                    return getter() > threshold;
                });
            } else {
                addBuildWarning("Unknown stop condition type '" + type + "'. Supported types: component_field_below, component_field_above. Please fix the configuration.");
                success = false;
            }
        }

        return success;
    }

    void buildEnvironment() {
        const auto& envConfig = config_.root()["environment"];
        if (envConfig.isNull()) {
            LOG_INFO("No environment configuration, skipping");
            return;
        }
        
        environment_.id = "environment";
        LOG_INFO("Building environment entity");
        
        buildServices(envConfig["services"], environment_.services, "env", "env");
        
        auto& registry = simulator_.getRegistry();
        auto& factory = ComponentFactory::instance();
        
        const auto& components = envConfig["components"];
        for (size_t i = 0; i < components.size(); ++i) {
            const auto& comp_config = components[i];
            std::string type_name = comp_config["type"].asString();
            std::string base_name = comp_config["name"].asString();
            
            std::string name = "env." + base_name;
            
            if (type_name.empty() || base_name.empty()) {
                addBuildError("Environment component at index " + std::to_string(i) +
                              " is missing a type or name. Please provide both fields in the configuration.");
                continue;
            }
            if (!factory.hasType(type_name)) {
                addBuildError(buildUnknownTypeMessage(type_name, name, factory, "environment configuration"));
                continue;
            }
            if (registry.has(name)) {
                addBuildError("Component name '" + name +
                              "' is already registered. Please use a unique environment component name.");
                continue;
            }
            
            auto component = factory.create(type_name);
            auto* comp_ptr = component.get();
            annotateComponentMetadata(comp_ptr, type_name, factory);
            
            const auto& component_config = comp_config["config"];
            component_config.resetAccessTracking();
            comp_ptr->configure(component_config);
            checkUnusedConfigKeys(name, type_name, component_config);
            
            comp_ptr->injectServices(globalServices_);
            comp_ptr->injectServices(environment_.services);
            
            environment_.components.push_back(comp_ptr);
            
            auto interfaces = factory.getInterfaces(type_name);
            registry.addDynamic(name, std::move(component), interfaces);
            
            LOG_INFO("Environment component registered: {}", name);
        }
    }
    
    void buildSingleVehicle() {
        buildServices(config_.services(), globalServices_, "flight_vehicle", "");
        
        auto& registry = simulator_.getRegistry();
        auto& factory = ComponentFactory::instance();
        
        const auto& components = config_.components();
        for (size_t i = 0; i < components.size(); ++i) {
            const auto& comp_config = components[i];
            std::string type_name = comp_config["type"].asString();
            std::string name = comp_config["name"].asString();
            
            if (type_name.empty() || name.empty()) {
                addBuildError("Component at index " + std::to_string(i) +
                              " is missing a type or name. Please provide both fields in the configuration.");
                continue;
            }
            
            if (!factory.hasType(type_name)) {
                addBuildError(buildUnknownTypeMessage(type_name, name, factory));
                continue;
            }

            if (registry.has(name)) {
                addBuildError("Component name '" + name +
                              "' is already registered. Please use a unique component name.");
                continue;
            }
            
            auto component = factory.create(type_name);
            auto* comp_ptr = component.get();
            annotateComponentMetadata(comp_ptr, type_name, factory);
            
            const auto& component_config = comp_config["config"];
            component_config.resetAccessTracking();
            comp_ptr->configure(component_config);
            checkUnusedConfigKeys(name, type_name, component_config);
            
            comp_ptr->injectServices(globalServices_);
            
            auto interfaces = factory.getInterfaces(type_name);
            registry.addDynamic(name, std::move(component), interfaces);
        }
    }
    
    void buildMultiVehicle() {
        const auto& vehiclesConfig = config_.vehicles();
        vehicles_.reserve(vehiclesConfig.size());
        for (size_t i = 0; i < vehiclesConfig.size(); ++i) {
            const auto& vConfig = vehiclesConfig[i];
            
            VehicleInstance vehicle;
            vehicle.id = vConfig["id"].asString();
            
            if (vehicle.id.empty()) {
                addBuildError("Vehicle at index " + std::to_string(i) +
                              " is missing an 'id'. Please provide a unique vehicle id.");
                continue;
            }
            
            LOG_INFO("Building vehicle: {}", vehicle.id);
            
            auto* pending_vehicle_services = &vehicle.services;
            buildServices(vConfig["services"], vehicle.services, vehicle.id, vehicle.id);
            
            auto& registry = simulator_.getRegistry();
            auto& factory = ComponentFactory::instance();
            
            const auto& components = vConfig["components"];
            for (size_t j = 0; j < components.size(); ++j) {
                const auto& comp_config = components[j];
                std::string type_name = comp_config["type"].asString();
                std::string base_name = comp_config["name"].asString();
                std::string name = vehicle.id + "." + base_name;
                
                if (type_name.empty() || base_name.empty()) {
                    addBuildError("Vehicle '" + vehicle.id + "' component at index " + std::to_string(j) +
                                  " is missing a type or name. Please provide both fields in the configuration.");
                    continue;
                }
                if (!factory.hasType(type_name)) {
                    addBuildError(buildUnknownTypeMessage(type_name, name, factory));
                    continue;
                }

                if (registry.has(name)) {
                    addBuildError("Component name '" + name +
                                  "' is already registered. Please use a unique component name within the vehicle scope.");
                    continue;
                }
                
                auto component = factory.create(type_name);
                auto* comp_ptr = component.get();
                annotateComponentMetadata(comp_ptr, type_name, factory);
                
                const auto& component_config = comp_config["config"];
                component_config.resetAccessTracking();
                comp_ptr->configure(component_config);
                checkUnusedConfigKeys(name, type_name, component_config);
                
                comp_ptr->injectServices(globalServices_);
                comp_ptr->injectServices(environment_.services);
                comp_ptr->injectServices(vehicle.services);
                
                vehicle.components.push_back(comp_ptr);
                
                auto interfaces = factory.getInterfaces(type_name);
                registry.addDynamic(name, std::move(component), interfaces);
            }
            
            vehicles_.push_back(std::move(vehicle));
            auto* stable_vehicle_services = &vehicles_.back().services;
            for (auto& pending : pending_coordinate_installations_) {
                if (pending.context == pending_vehicle_services) {
                    pending.context = stable_vehicle_services;
                }
            }
        }
    }
    
    void buildServices(const ConfigNode& svcConfig, ServiceContext& context) {
        if (svcConfig.isNull()) return;
        
        // 坐标服务
        if (svcConfig.has("coordinate")) {
            const auto& coordConfig = svcConfig["coordinate"];
            if (coordConfig["enabled"].asBool(false)) {
                auto coordService = std::make_shared<gnc::services::CoordinateService>();
                context.registerService(coordService);
                LOG_INFO("CoordinateService enabled");
            }
        }
        
        // 未来可扩展更多服务...
        // if (svcConfig.has("ephemeris")) { ... }
        // if (svcConfig.has("atmosphere")) { ... }
    }
    
    struct PendingCoordinateInstallation {
        ServiceContext* context = nullptr;
        ConfigNode config;
        std::string service_scope_name;
        std::string registry_scope;
    };

    void bindCoordinateServices() {
        auto& registry = simulator_.getRegistry();
        for (const auto& pending : pending_coordinate_installations_) {
            if (!pending.context) {
                continue;
            }

            auto* service = pending.context->get<gnc::services::CoordinateService>();
            if (!service) {
                continue;
            }

            try {
                ScopedRegistry scoped(pending.registry_scope, registry, "coordinate_service");
                gnc::services::coordinate::soviet::installSovietCoordinateSystem(
                    *service,
                    pending.config,
                    scoped,
                    pending.service_scope_name);
            } catch (const std::exception& e) {
                addBuildError("Coordinate service installation failed in scope '" +
                              pending.service_scope_name + "': " + e.what());
            }
        }
    }

    void recordCoordinateInstallation(ServiceContext& context,
                                      const ConfigNode& config,
                                      const std::string& service_scope_name,
                                      const std::string& registry_scope) {
        for (auto& pending : pending_coordinate_installations_) {
            if (pending.context == &context) {
                pending.config = config;
                pending.service_scope_name = service_scope_name;
                pending.registry_scope = registry_scope;
                return;
            }
        }

        pending_coordinate_installations_.push_back(
            PendingCoordinateInstallation{&context, config, service_scope_name, registry_scope});
    }

    void buildServices(const ConfigNode& svcConfig,
                       ServiceContext& context,
                       const std::string& service_scope_name,
                       const std::string& registry_scope) {
        if (svcConfig.isNull()) return;

        if (svcConfig.has("coordinate")) {
            const auto& coordConfig = svcConfig["coordinate"];
            if (coordConfig["enabled"].asBool(false)) {
                const std::string scheme = normalizeCoordinateSchemeName(
                    coordConfig["scheme"].asString("soviet"));
                if (!scheme.empty() && scheme != "soviet") {
                    addBuildError("Unknown coordinate service scheme '" + scheme +
                                  "'. Only 'soviet' is supported.");
                } else {
                    context.registerService(std::make_shared<gnc::services::CoordinateService>());
                    recordCoordinateInstallation(context, coordConfig, service_scope_name, registry_scope);
                    LOG_INFO("CoordinateService enabled with built-in Soviet coordinate system");
                }
            }
        }
    }

    ConfigManager config_;
    Simulator simulator_;
    ServiceContext globalServices_;
    EnvironmentInstance environment_;
    std::vector<VehicleInstance> vehicles_;
    std::vector<PendingCoordinateInstallation> pending_coordinate_installations_;
    std::vector<std::string> build_errors_;
    std::vector<std::string> build_warnings_;
};

} // namespace gnc::core
