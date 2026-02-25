/**
 * @file config_manager.hpp
 * @brief 配置管理器
 * 
 * 支持从 JSON 文件加载仿真配置
 */
#pragma once

#include "gnc/common/logger.hpp"
#include <string>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <vector>
#include <stdexcept>

namespace gnc::core {

/**
 * @brief 简单的配置节点
 * 
 * 轻量级 JSON 解析，避免外部依赖
 * 支持基本类型：string, number, bool, array, object
 */
class ConfigNode {
public:
    enum class Type { Null, Bool, Number, String, Array, Object };
    
    ConfigNode() : type_(Type::Null) {}
    
    // 基本类型获取
    bool isNull() const { return type_ == Type::Null; }
    bool isBool() const { return type_ == Type::Bool; }
    bool isNumber() const { return type_ == Type::Number; }
    bool isString() const { return type_ == Type::String; }
    bool isArray() const { return type_ == Type::Array; }
    bool isObject() const { return type_ == Type::Object; }
    
    // 值获取
    bool asBool(bool default_val = false) const {
        return isBool() ? bool_val_ : default_val;
    }
    
    double asDouble(double default_val = 0.0) const {
        return isNumber() ? num_val_ : default_val;
    }
    
    int asInt(int default_val = 0) const {
        return isNumber() ? static_cast<int>(num_val_) : default_val;
    }
    
    const std::string& asString(const std::string& default_val = "") const {
        static std::string empty;
        return isString() ? str_val_ : (default_val.empty() ? empty : default_val);
    }
    
    // 对象访问
    const ConfigNode& operator[](const std::string& key) const {
        static ConfigNode null_node;
        if (!isObject()) return null_node;
        auto it = obj_val_.find(key);
        return it != obj_val_.end() ? it->second : null_node;
    }
    
    bool has(const std::string& key) const {
        return isObject() && obj_val_.count(key) > 0;
    }
    
    // 数组访问
    const ConfigNode& operator[](size_t index) const {
        static ConfigNode null_node;
        if (!isArray() || index >= arr_val_.size()) return null_node;
        return arr_val_[index];
    }
    
    size_t size() const {
        if (isArray()) return arr_val_.size();
        if (isObject()) return obj_val_.size();
        return 0;
    }
    
    // 迭代器（用于遍历对象）
    auto begin() const { return obj_val_.begin(); }
    auto end() const { return obj_val_.end(); }
    
    // 构造函数
    static ConfigNode makeBool(bool val) {
        ConfigNode n; n.type_ = Type::Bool; n.bool_val_ = val; return n;
    }
    static ConfigNode makeNumber(double val) {
        ConfigNode n; n.type_ = Type::Number; n.num_val_ = val; return n;
    }
    static ConfigNode makeString(const std::string& val) {
        ConfigNode n; n.type_ = Type::String; n.str_val_ = val; return n;
    }
    static ConfigNode makeArray() {
        ConfigNode n; n.type_ = Type::Array; return n;
    }
    static ConfigNode makeObject() {
        ConfigNode n; n.type_ = Type::Object; return n;
    }
    
    void push(const ConfigNode& node) {
        if (type_ == Type::Array) arr_val_.push_back(node);
    }
    
    void set(const std::string& key, const ConfigNode& node) {
        if (type_ == Type::Object) obj_val_[key] = node;
    }
    
private:
    Type type_;
    bool bool_val_ = false;
    double num_val_ = 0.0;
    std::string str_val_;
    std::vector<ConfigNode> arr_val_;
    std::unordered_map<std::string, ConfigNode> obj_val_;
};

/**
 * @brief 简单 JSON 解析器
 */
class JsonParser {
public:
    static ConfigNode parse(const std::string& json) {
        size_t pos = 0;
        return parseValue(json, pos);
    }
    
    static ConfigNode parseFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            LOG_ERROR("Failed to open config file: {}", filename);
            return ConfigNode();
        }
        std::stringstream buffer;
        buffer << file.rdbuf();
        return parse(buffer.str());
    }
    
private:
    static void skipWhitespace(const std::string& json, size_t& pos) {
        while (pos < json.size() && std::isspace(json[pos])) pos++;
    }
    
    static ConfigNode parseValue(const std::string& json, size_t& pos) {
        skipWhitespace(json, pos);
        if (pos >= json.size()) return ConfigNode();
        
        char c = json[pos];
        if (c == '{') return parseObject(json, pos);
        if (c == '[') return parseArray(json, pos);
        if (c == '"') return parseString(json, pos);
        if (c == 't' || c == 'f') return parseBool(json, pos);
        if (c == 'n') { pos += 4; return ConfigNode(); } // null
        if (c == '-' || std::isdigit(c)) return parseNumber(json, pos);
        
        return ConfigNode();
    }
    
    static ConfigNode parseObject(const std::string& json, size_t& pos) {
        auto node = ConfigNode::makeObject();
        pos++; // skip '{'
        skipWhitespace(json, pos);
        
        while (pos < json.size() && json[pos] != '}') {
            skipWhitespace(json, pos);
            if (json[pos] == '"') {
                auto key = parseString(json, pos).asString();
                skipWhitespace(json, pos);
                if (json[pos] == ':') pos++;
                skipWhitespace(json, pos);
                auto value = parseValue(json, pos);
                node.set(key, value);
            }
            skipWhitespace(json, pos);
            if (json[pos] == ',') pos++;
        }
        if (pos < json.size()) pos++; // skip '}'
        return node;
    }
    
    static ConfigNode parseArray(const std::string& json, size_t& pos) {
        auto node = ConfigNode::makeArray();
        pos++; // skip '['
        skipWhitespace(json, pos);
        
        while (pos < json.size() && json[pos] != ']') {
            node.push(parseValue(json, pos));
            skipWhitespace(json, pos);
            if (json[pos] == ',') pos++;
        }
        if (pos < json.size()) pos++; // skip ']'
        return node;
    }
    
    static ConfigNode parseString(const std::string& json, size_t& pos) {
        pos++; // skip opening '"'
        std::string result;
        while (pos < json.size() && json[pos] != '"') {
            if (json[pos] == '\\' && pos + 1 < json.size()) {
                pos++;
                switch (json[pos]) {
                    case 'n': result += '\n'; break;
                    case 't': result += '\t'; break;
                    case '"': result += '"'; break;
                    case '\\': result += '\\'; break;
                    default: result += json[pos]; break;
                }
            } else {
                result += json[pos];
            }
            pos++;
        }
        if (pos < json.size()) pos++; // skip closing '"'
        return ConfigNode::makeString(result);
    }
    
    static ConfigNode parseNumber(const std::string& json, size_t& pos) {
        size_t start = pos;
        if (json[pos] == '-') pos++;
        while (pos < json.size() && (std::isdigit(json[pos]) || json[pos] == '.' || 
               json[pos] == 'e' || json[pos] == 'E' || json[pos] == '+' || json[pos] == '-')) {
            pos++;
        }
        double val = std::stod(json.substr(start, pos - start));
        return ConfigNode::makeNumber(val);
    }
    
    static ConfigNode parseBool(const std::string& json, size_t& pos) {
        if (json.substr(pos, 4) == "true") {
            pos += 4;
            return ConfigNode::makeBool(true);
        }
        if (json.substr(pos, 5) == "false") {
            pos += 5;
            return ConfigNode::makeBool(false);
        }
        return ConfigNode();
    }
};

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
