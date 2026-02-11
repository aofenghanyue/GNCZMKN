/**
 * @file config_manager.hpp
 * @brief 配置管理器
 * 
 * 支持从 JSON 文件加载仿真配置
 */
#pragma once

#include "config_node.hpp"
#include "json_parser.hpp"
#include "gnc/common/logger.hpp"
#include <string>
#include <optional>

namespace gnc::core {

/**
 * @brief 配置管理器
 * 
 * 管理仿真配置，包括：
 * - 仿真参数 (步长、时长等)
 * - 组件列表及其配置
 */
class ConfigManager {
public:
    ConfigManager() = default;
    
    /// 从 JSON 文件加载配置
    bool loadFromFile(const std::string& filename) {
        config_ = JsonParser::parseFile(filename);
        if (config_.isNull()) {
            LOG_ERROR("Failed to parse config file: {}", filename);
            return false;
        }
        LOG_INFO("Configuration loaded from: {}", filename);
        return true;
    }
    
    /// 从 JSON 字符串加载配置
    bool loadFromString(const std::string& json) {
        config_ = JsonParser::parse(json);
        return !config_.isNull();
    }
    
    /// 获取根配置节点
    const ConfigNode& root() const { return config_; }
    
    /// 获取仿真配置
    const ConfigNode& simulation() const { return config_["simulation"]; }
    
    /// 获取组件配置列表
    const ConfigNode& components() const { return config_["components"]; }
    
    /// 获取指定组件的配置
    const ConfigNode& componentConfig(const std::string& name) const {
        const auto& comps = components();
        for (size_t i = 0; i < comps.size(); ++i) {
            if (comps[i]["name"].asString() == name) {
                return comps[i]["config"];
            }
        }
        static ConfigNode null_node;
        return null_node;
    }
    
    /// 获取服务配置（显式配置驱动）
    const ConfigNode& services() const { return config_["services"]; }
    
    /// 获取飞行器列表配置（多飞行器支持）
    const ConfigNode& vehicles() const { return config_["vehicles"]; }
    
    /// 获取全局服务配置
    const ConfigNode& globalServices() const { return config_["global_services"]; }
    
    /// 检查是否为多飞行器模式
    bool isMultiVehicle() const { return config_.has("vehicles"); }
    
private:
    ConfigNode config_;
};

} // namespace gnc::core
