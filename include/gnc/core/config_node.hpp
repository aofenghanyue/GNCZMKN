/**
 * @file config_node.hpp
 * @brief 配置节点
 *
 * 轻量级 JSON 节点表示，避免外部依赖
 */
#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>

namespace gnc::core {

/**
 * @brief 简单的配置节点
 *
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

} // namespace gnc::core
