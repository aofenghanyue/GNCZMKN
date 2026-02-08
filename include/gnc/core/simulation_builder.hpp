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
        // 1. 配置仿真参数
        const auto& sim_config = config_.simulation();
        SimulatorConfig cfg;
        cfg.dt = sim_config["dt"].asDouble(0.01);
        cfg.duration = sim_config["duration"].asDouble(10.0);
        simulator_.configure(cfg);
        
        LOG_INFO("Simulation config: dt={}, duration={}", cfg.dt, cfg.duration);
        
        // 2. 构建全局服务（优先级最高）
        buildServices(config_.globalServices(), globalServices_);
        
        // 3. 构建 Environment（先于Vehicle）
        buildEnvironment();
        
        // 4. 判断模式并构建Vehicle
        if (config_.isMultiVehicle()) {
            buildMultiVehicle();
        } else {
            buildSingleVehicle();
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
    /// 构建Environment实体
    void buildEnvironment() {
        const auto& envConfig = config_.root()["environment"];
        if (envConfig.isNull()) {
            LOG_INFO("No environment configuration, skipping");
            return;
        }
        
        environment_.id = "environment";
        LOG_INFO("Building environment entity");
        
        // 创建Environment专有服务
        buildServices(envConfig["services"], environment_.services);
        
        // 创建Environment组件（将注册到全局Registry，但标记为环境组件）
        auto& registry = simulator_.getRegistry();
        auto& factory = ComponentFactory::instance();
        
        const auto& components = envConfig["components"];
        for (size_t i = 0; i < components.size(); ++i) {
            const auto& comp_config = components[i];
            std::string type_name = comp_config["type"].asString();
            std::string base_name = comp_config["name"].asString();
            
            // 组件名添加env.前缀
            std::string name = "env." + base_name;
            
            if (type_name.empty() || base_name.empty()) continue;
            if (!factory.hasType(type_name)) {
                LOG_ERROR("Unknown component type: {}", type_name);
                continue;
            }
            
            auto component = factory.create(type_name);
            auto* comp_ptr = component.get();
            
            // 注入服务（全局 + 环境专有）
            comp_ptr->injectServices(globalServices_);
            comp_ptr->injectServices(environment_.services);
            
            environment_.components.push_back(comp_ptr);
            
            auto interfaces = factory.getInterfaces(type_name);
            registry.addDynamic(name, std::move(component), interfaces);
            
            LOG_INFO("Environment component registered: {}", name);
        }
    }
    
    /// 构建单飞行器模式
    void buildSingleVehicle() {
        // 创建服务
        buildServices(config_.services(), globalServices_);
        
        // 创建组件
        auto& registry = simulator_.getRegistry();
        auto& factory = ComponentFactory::instance();
        
        const auto& components = config_.components();
        for (size_t i = 0; i < components.size(); ++i) {
            const auto& comp_config = components[i];
            std::string type_name = comp_config["type"].asString();
            std::string name = comp_config["name"].asString();
            
            if (type_name.empty() || name.empty()) {
                LOG_WARNING("Skipping invalid component config at index {}", i);
                continue;
            }
            
            if (!factory.hasType(type_name)) {
                LOG_ERROR("Unknown component type: {}", type_name);
                continue;
            }
            
            // 创建组件
            auto component = factory.create(type_name);
            auto* comp_ptr = component.get();
            
            // 注入服务
            comp_ptr->injectServices(globalServices_);
            
            // 获取接口列表用于注册
            auto interfaces = factory.getInterfaces(type_name);
            registry.addDynamic(name, std::move(component), interfaces);
        }
    }
    
    /// 构建多飞行器模式
    void buildMultiVehicle() {
        // 创建各飞行器
        const auto& vehiclesConfig = config_.vehicles();
        for (size_t i = 0; i < vehiclesConfig.size(); ++i) {
            const auto& vConfig = vehiclesConfig[i];
            
            VehicleInstance vehicle;
            vehicle.id = vConfig["id"].asString();
            
            if (vehicle.id.empty()) {
                LOG_WARNING("Skipping vehicle with no ID at index {}", i);
                continue;
            }
            
            LOG_INFO("Building vehicle: {}", vehicle.id);
            
            // 创建飞行器专有服务
            buildServices(vConfig["services"], vehicle.services);
            
            // 创建飞行器组件
            auto& registry = simulator_.getRegistry();
            auto& factory = ComponentFactory::instance();
            
            const auto& components = vConfig["components"];
            for (size_t j = 0; j < components.size(); ++j) {
                const auto& comp_config = components[j];
                std::string type_name = comp_config["type"].asString();
                std::string base_name = comp_config["name"].asString();
                
                // 组件名添加飞行器前缀
                std::string name = vehicle.id + "." + base_name;
                
                if (type_name.empty() || base_name.empty()) continue;
                if (!factory.hasType(type_name)) {
                    LOG_ERROR("Unknown component type: {}", type_name);
                    continue;
                }
                
                auto component = factory.create(type_name);
                auto* comp_ptr = component.get();
                
                // 注入服务（全局 → 环境 → 飞行器专有）
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
    
    /// 根据配置构建服务
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
};

} // namespace gnc::core
