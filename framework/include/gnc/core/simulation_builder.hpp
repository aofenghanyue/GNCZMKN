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
#include "service_context.hpp"
#include "gnc/common/logger.hpp"
#include "gnc/services/coordinate/coordinate_service.hpp"
#include <sstream>

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

        const auto& sim_config = config_.simulation();
        SimulatorConfig cfg;
        cfg.dt = sim_config["dt"].asDouble(0.01);
        cfg.duration = sim_config["duration"].asDouble(10.0);
        simulator_.configure(cfg);
        
        LOG_INFO("Simulation config: dt={}, duration={}", cfg.dt, cfg.duration);
        
        buildServices(config_.globalServices(), globalServices_);
        
        buildEnvironment();
        
        if (config_.isMultiVehicle()) {
            buildMultiVehicle();
        } else {
            buildSingleVehicle();
        }
        
        auto validation = DependencyValidator::validate(simulator_.getRegistry());
        for (const auto& error : validation.errors) {
            addBuildError(error);
        }
        for (const auto& warning : validation.warnings) {
            addBuildWarning(warning);
        }

        if (!reportBuildDiagnostics()) {
            throw std::runtime_error("Simulation build failed with " +
                                     std::to_string(build_errors_.size()) +
                                     " error(s). See diagnostics report above.");
        }
        
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
    static std::string joinStrings(const std::vector<std::string>& values) {
        std::string result;
        for (size_t i = 0; i < values.size(); ++i) {
            if (i > 0) result += ", ";
            result += values[i];
        }
        return result.empty() ? "(none)" : result;
    }

    static std::string joinRegisteredTypes(const ComponentFactory& factory) {
        return joinStrings(factory.getRegisteredTypes());
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

    void buildEnvironment() {
        const auto& envConfig = config_.root()["environment"];
        if (envConfig.isNull()) {
            LOG_INFO("No environment configuration, skipping");
            return;
        }
        
        environment_.id = "environment";
        LOG_INFO("Building environment entity");
        
        buildServices(envConfig["services"], environment_.services);
        
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
                addBuildError("Unknown component type: '" + type_name +
                              "' (component name: '" + name + "') in environment configuration. "
                              "Available types: " + joinRegisteredTypes(factory) +
                              ". Please register the component or fix the type name.");
                continue;
            }
            if (registry.has(name)) {
                addBuildError("Component name '" + name +
                              "' is already registered. Please use a unique environment component name.");
                continue;
            }
            
            auto component = factory.create(type_name);
            auto* comp_ptr = component.get();
            
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
        buildServices(config_.services(), globalServices_);
        
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
                addBuildError("Unknown component type: '" + type_name +
                              "' (component name: '" + name + "'). Available types: " +
                              joinRegisteredTypes(factory) +
                              ". Please register the component or fix the type name.");
                continue;
            }

            if (registry.has(name)) {
                addBuildError("Component name '" + name +
                              "' is already registered. Please use a unique component name.");
                continue;
            }
            
            auto component = factory.create(type_name);
            auto* comp_ptr = component.get();
            
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
            
            buildServices(vConfig["services"], vehicle.services);
            
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
                    addBuildError("Unknown component type: '" + type_name +
                                  "' (component name: '" + name + "'). Available types: " +
                                  joinRegisteredTypes(factory) +
                                  ". Please register the component or fix the type name.");
                    continue;
                }

                if (registry.has(name)) {
                    addBuildError("Component name '" + name +
                                  "' is already registered. Please use a unique component name within the vehicle scope.");
                    continue;
                }
                
                auto component = factory.create(type_name);
                auto* comp_ptr = component.get();
                
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
    
    ConfigManager config_;
    Simulator simulator_;
    ServiceContext globalServices_;
    EnvironmentInstance environment_;
    std::vector<VehicleInstance> vehicles_;
    std::vector<std::string> build_errors_;
    std::vector<std::string> build_warnings_;
};

} // namespace gnc::core
