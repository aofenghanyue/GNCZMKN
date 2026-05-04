/**
 * @file config_manager.hpp
 * @brief 配置管理器
 * 
 * 支持从 JSON 文件加载仿真配置
 */
#pragma once

#include "gnc/common/logger.hpp"
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

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
    
    std::string asString(const std::string& default_val = "") const {
        return isString() ? str_val_ : default_val;
    }
    
    // 对象访问
    const ConfigNode& operator[](const std::string& key) const {
        static ConfigNode null_node;
        if (!isObject()) return null_node;
        accessed_keys_.insert(key);
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

    std::vector<std::string> getUnusedKeys() const {
        std::vector<std::string> unused;
        if (!isObject()) return unused;
        for (const auto& [key, _] : obj_val_) {
            if (!key.empty() && key[0] == '_') continue;
            if (accessed_keys_.find(key) == accessed_keys_.end()) {
                unused.push_back(key);
            }
        }
        return unused;
    }

    void resetAccessTracking() const {
        accessed_keys_.clear();
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
    mutable std::unordered_set<std::string> accessed_keys_;
};

/**
 * @brief 简单 JSON 解析器
 */
class IConfigParser {
public:
    virtual ~IConfigParser() = default;

    virtual ConfigNode parseFile(const std::string& filename) = 0;
    virtual ConfigNode parseString(const std::string& text) = 0;
};

class BuiltinJsonConfigParser final : public IConfigParser {
public:
    ConfigNode parseString(const std::string& json) override {
        const std::string normalized = normalizeJsonWithComments(json);
        size_t pos = 0;
        return parseValue(normalized, pos);
    }
    
    ConfigNode parseFile(const std::string& filename) override {
        std::ifstream file(filename);
        if (!file.is_open()) {
            LOG_ERROR("Failed to open config file: {}", filename);
            return ConfigNode();
        }
        std::stringstream buffer;
        buffer << file.rdbuf();
        return parseString(buffer.str());
    }
    
private:
    static std::string normalizeJsonWithComments(const std::string& json) {
        std::string normalized = json;
        bool in_string = false;
        bool escaped = false;

        for (size_t i = 0; i < normalized.size(); ++i) {
            const char c = normalized[i];
            if (in_string) {
                if (escaped) {
                    escaped = false;
                } else if (c == '\\') {
                    escaped = true;
                } else if (c == '"') {
                    in_string = false;
                }
                continue;
            }

            if (c == '"') {
                in_string = true;
                continue;
            }

            if (c == '/' && i + 1 < normalized.size() && normalized[i + 1] == '/') {
                normalized[i] = ' ';
                normalized[i + 1] = ' ';
                i += 2;
                while (i < normalized.size() && normalized[i] != '\n' &&
                       normalized[i] != '\r') {
                    normalized[i] = ' ';
                    ++i;
                }
                if (i > 0) {
                    --i;
                }
                continue;
            }

            if (c == '/' && i + 1 < normalized.size() && normalized[i + 1] == '*') {
                normalized[i] = ' ';
                normalized[i + 1] = ' ';
                i += 2;
                while (i + 1 < normalized.size() &&
                       !(normalized[i] == '*' && normalized[i + 1] == '/')) {
                    if (normalized[i] != '\n' && normalized[i] != '\r') {
                        normalized[i] = ' ';
                    }
                    ++i;
                }
                if (i + 1 < normalized.size()) {
                    normalized[i] = ' ';
                    normalized[i + 1] = ' ';
                    ++i;
                }
            }
        }

        in_string = false;
        escaped = false;
        for (size_t i = 0; i < normalized.size(); ++i) {
            const char c = normalized[i];
            if (in_string) {
                if (escaped) {
                    escaped = false;
                } else if (c == '\\') {
                    escaped = true;
                } else if (c == '"') {
                    in_string = false;
                }
                continue;
            }

            if (c == '"') {
                in_string = true;
                continue;
            }

            if (c != ',') {
                continue;
            }

            size_t next = i + 1;
            while (next < normalized.size() &&
                   std::isspace(static_cast<unsigned char>(normalized[next]))) {
                ++next;
            }
            if (next < normalized.size() &&
                (normalized[next] == '}' || normalized[next] == ']')) {
                normalized[i] = ' ';
            }
        }

        return normalized;
    }

    static void skipWhitespace(const std::string& json, size_t& pos) {
        while (pos < json.size() &&
               std::isspace(static_cast<unsigned char>(json[pos]))) {
            pos++;
        }
    }
    
    static ConfigNode parseValue(const std::string& json, size_t& pos) {
        skipWhitespace(json, pos);
        if (pos >= json.size()) return ConfigNode();
        
        char c = json[pos];
        if (c == '{') return parseObject(json, pos);
        if (c == '[') return parseArray(json, pos);
        if (c == '"') return parseStringToken(json, pos);
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
                auto key = parseStringToken(json, pos).asString();
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
    
    static ConfigNode parseStringToken(const std::string& json, size_t& pos) {
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
 * @brief Expands filesystem includes and merges configuration fragments.
 */
class ConfigPreprocessor {
public:
    ConfigPreprocessor(IConfigParser& parser, const std::string& root_file)
        : parser_(parser) {
        std::error_code ec;
        root_file_ = std::filesystem::absolute(std::filesystem::path(root_file), ec);
        if (ec) {
            root_file_ = std::filesystem::path(root_file);
        }
        root_file_ = root_file_.lexically_normal();
        repo_root_ = findRepoRoot(root_file_);
        project_root_ = findProjectRoot(root_file_, repo_root_);
    }

    ConfigNode preprocess(const ConfigNode& root) {
        std::vector<std::string> include_stack;
        include_stack.push_back(stackKey(root_file_));
        return preprocessNode(root, root_file_, include_stack, "$");
    }

    static bool containsIncludeDirective(const ConfigNode& node) {
        if (node.isObject()) {
            if (node.has("$include")) {
                return true;
            }
            for (const auto& entry : node) {
                if (containsIncludeDirective(entry.second)) {
                    return true;
                }
            }
            return false;
        }
        if (node.isArray()) {
            for (size_t i = 0; i < node.size(); ++i) {
                if (containsIncludeDirective(node[i])) {
                    return true;
                }
            }
        }
        return false;
    }

private:
    using fs_path = std::filesystem::path;

    static std::string normalize(const fs_path& path) {
        return path.lexically_normal().generic_string();
    }

    static bool startsWith(const std::string& text, const std::string& prefix) {
        return text.rfind(prefix, 0) == 0;
    }

    static fs_path findRepoRoot(const fs_path& config_file) {
        std::error_code ec;
        fs_path current = config_file.has_parent_path()
                              ? config_file.parent_path()
                              : std::filesystem::current_path(ec);
        if (ec) {
            current = ".";
        }
        current = current.lexically_normal();

        while (!current.empty()) {
            ec.clear();
            const bool has_cmake =
                std::filesystem::exists(current / "CMakeLists.txt", ec) && !ec;
            ec.clear();
            const bool has_framework =
                std::filesystem::exists(current / "framework", ec) && !ec;
            if (has_cmake && has_framework) {
                return current;
            }

            const fs_path parent = current.parent_path();
            if (parent == current) {
                break;
            }
            current = parent;
        }

        ec.clear();
        return std::filesystem::current_path(ec).lexically_normal();
    }

    static fs_path findProjectRoot(const fs_path& config_file,
                                   const fs_path& repo_root) {
        std::error_code ec;
        const fs_path relative = std::filesystem::relative(config_file, repo_root, ec);
        if (ec) {
            return {};
        }

        auto it = relative.begin();
        if (it == relative.end() || it->generic_string() != "user") {
            return {};
        }
        ++it;
        if (it == relative.end()) {
            return {};
        }

        const std::string project_name = it->generic_string();
        if (project_name.empty() || project_name == "data" ||
            project_name == "outputs") {
            return {};
        }

        const fs_path candidate = repo_root / "user" / project_name;
        ec.clear();
        if (!std::filesystem::is_directory(candidate, ec) || ec) {
            return {};
        }
        return candidate.lexically_normal();
    }

    static std::string stackKey(const fs_path& path) {
        std::error_code ec;
        fs_path absolute = std::filesystem::absolute(path, ec);
        if (ec) {
            absolute = path;
        }

        ec.clear();
        const fs_path canonical = std::filesystem::weakly_canonical(absolute, ec);
        if (!ec) {
            return canonical.lexically_normal().generic_string();
        }
        return absolute.lexically_normal().generic_string();
    }

    std::vector<std::string> readIncludeList(const ConfigNode& include_node,
                                             const std::string& node_path) const {
        std::vector<std::string> includes;
        if (include_node.isString()) {
            if (include_node.asString().empty()) {
                throw std::runtime_error(node_path +
                                         ".$include must not be an empty string.");
            }
            includes.push_back(include_node.asString());
            return includes;
        }
        if (include_node.isArray()) {
            for (size_t i = 0; i < include_node.size(); ++i) {
                const auto& item = include_node[i];
                if (!item.isString() || item.asString().empty()) {
                    throw std::runtime_error(node_path + ".$include[" +
                                             std::to_string(i) +
                                             "] must be a non-empty string.");
                }
                includes.push_back(item.asString());
            }
            return includes;
        }
        throw std::runtime_error(node_path +
                                 ".$include must be a string or an array of strings.");
    }

    fs_path resolveIncludePath(const std::string& include_path,
                               const fs_path& current_file,
                               const std::string& node_path) const {
        const auto path_after_scheme = [](const std::string& value,
                                          const std::string& scheme) {
            return value.substr(scheme.size());
        };

        fs_path resolved;
        if (startsWith(include_path, "repo://")) {
            resolved = repo_root_ / path_after_scheme(include_path, "repo://");
        } else if (startsWith(include_path, "project://")) {
            if (project_root_.empty()) {
                throw std::runtime_error(
                    node_path +
                    ".$include uses project:// but the config file is not under user/<project>/.");
            }
            resolved = project_root_ / path_after_scheme(include_path, "project://");
        } else if (startsWith(include_path, "user-data://")) {
            resolved = repo_root_ / "user" / "data" /
                       path_after_scheme(include_path, "user-data://");
        } else {
            const fs_path requested(include_path);
            resolved = requested.is_absolute() ? requested
                                               : current_file.parent_path() / requested;
        }

        std::error_code ec;
        resolved = std::filesystem::absolute(resolved, ec);
        if (ec) {
            resolved = fs_path(resolved);
        }
        resolved = resolved.lexically_normal();

        ec.clear();
        if (!std::filesystem::exists(resolved, ec) || ec) {
            throw std::runtime_error(node_path + ".$include file '" +
                                     include_path + "' was not found at '" +
                                     normalize(resolved) + "'.");
        }
        ec.clear();
        if (!std::filesystem::is_regular_file(resolved, ec) || ec) {
            throw std::runtime_error(node_path + ".$include file '" +
                                     include_path + "' is not a regular file.");
        }
        return resolved;
    }

    ConfigNode loadIncludedFile(const fs_path& include_file,
                                std::vector<std::string>& include_stack,
                                const std::string& node_path) {
        const std::string key = stackKey(include_file);
        if (std::find(include_stack.begin(), include_stack.end(), key) !=
            include_stack.end()) {
            throw std::runtime_error(node_path +
                                     ".$include cycle detected at '" +
                                     normalize(include_file) + "'.");
        }

        include_stack.push_back(key);
        ConfigNode parsed = parser_.parseFile(include_file.string());
        if (parsed.isNull()) {
            include_stack.pop_back();
            throw std::runtime_error(node_path +
                                     ".$include failed to parse file '" +
                                     normalize(include_file) + "'.");
        }
        ConfigNode expanded =
            preprocessNode(parsed, include_file, include_stack, normalize(include_file));
        include_stack.pop_back();
        return expanded;
    }

    static ConfigNode deepMerge(const ConfigNode& base,
                                const ConfigNode& overlay) {
        if (!base.isObject() || !overlay.isObject()) {
            return overlay;
        }

        ConfigNode merged = ConfigNode::makeObject();
        for (const auto& [key, value] : base) {
            merged.set(key, value);
        }
        for (const auto& [key, value] : overlay) {
            if (base.has(key)) {
                merged.set(key, deepMerge(base[key], value));
            } else {
                merged.set(key, value);
            }
        }
        return merged;
    }

    static bool hasLocalFields(const ConfigNode& node) {
        for (const auto& entry : node) {
            if (entry.first != "$include") {
                return true;
            }
        }
        return false;
    }

    ConfigNode preprocessObjectWithInclude(const ConfigNode& node,
                                           const fs_path& current_file,
                                           std::vector<std::string>& include_stack,
                                           const std::string& node_path) {
        const auto includes = readIncludeList(node["$include"], node_path);
        const bool local_fields_present = hasLocalFields(node);

        ConfigNode merged;
        bool have_merged = false;
        for (const auto& include_path : includes) {
            const fs_path resolved =
                resolveIncludePath(include_path, current_file, node_path);
            ConfigNode included =
                loadIncludedFile(resolved, include_stack, node_path);
            if (!have_merged) {
                merged = included;
                have_merged = true;
                continue;
            }
            if (!merged.isObject() || !included.isObject()) {
                throw std::runtime_error(
                    node_path +
                    ".$include with multiple entries requires object documents.");
            }
            merged = deepMerge(merged, included);
        }

        if (!local_fields_present) {
            return merged;
        }
        if (!merged.isObject()) {
            throw std::runtime_error(
                node_path +
                ".$include with local fields requires included object documents.");
        }

        ConfigNode local = ConfigNode::makeObject();
        for (const auto& [key, child] : node) {
            if (key == "$include") {
                continue;
            }
            local.set(key,
                      preprocessNode(child,
                                     current_file,
                                     include_stack,
                                     node_path + "." + key));
        }
        return deepMerge(merged, local);
    }

    ConfigNode preprocessNode(const ConfigNode& node,
                              const fs_path& current_file,
                              std::vector<std::string>& include_stack,
                              const std::string& node_path) {
        if (node.isObject()) {
            if (node.has("$include")) {
                return preprocessObjectWithInclude(node,
                                                   current_file,
                                                   include_stack,
                                                   node_path);
            }

            ConfigNode object = ConfigNode::makeObject();
            for (const auto& [key, child] : node) {
                object.set(key,
                           preprocessNode(child,
                                          current_file,
                                          include_stack,
                                          node_path + "." + key));
            }
            return object;
        }

        if (node.isArray()) {
            ConfigNode array = ConfigNode::makeArray();
            for (size_t i = 0; i < node.size(); ++i) {
                array.push(preprocessNode(node[i],
                                          current_file,
                                          include_stack,
                                          node_path + "[" + std::to_string(i) + "]"));
            }
            return array;
        }

        return node;
    }

    IConfigParser& parser_;
    fs_path root_file_;
    fs_path repo_root_;
    fs_path project_root_;
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
    ConfigManager()
        : ConfigManager(std::make_unique<BuiltinJsonConfigParser>()) {}

    explicit ConfigManager(std::unique_ptr<IConfigParser> parser)
        : parser_(std::move(parser)) {
        if (!parser_) {
            parser_ = std::make_unique<BuiltinJsonConfigParser>();
        }
    }
    
    /// 从 JSON 文件加载配置
    bool loadFromFile(const std::string& filename) {
        ConfigNode parsed = parser_->parseFile(filename);
        if (parsed.isNull()) {
            LOG_ERROR("Failed to parse config file: {}", filename);
            return false;
        }
        try {
            ConfigPreprocessor preprocessor(*parser_, filename);
            config_ = preprocessor.preprocess(parsed);
        } catch (const std::exception& e) {
            LOG_ERROR("Failed to preprocess config file '{}': {}", filename, e.what());
            return false;
        }
        LOG_INFO("Configuration loaded from: {}", filename);
        return true;
    }
    
    /// 从 JSON 字符串加载配置
    bool loadFromString(const std::string& json) {
        config_ = parser_->parseString(json);
        if (config_.isNull()) {
            return false;
        }
        if (ConfigPreprocessor::containsIncludeDirective(config_)) {
            LOG_ERROR("Config loaded from string cannot use filesystem $include directives.");
            config_ = ConfigNode();
            return false;
        }
        return true;
    }
    
    /// 获取根配置节点
    const ConfigNode& root() const { return config_; }
    
    /// 获取仿真配置
    const ConfigNode& simulation() const { return config_["simulation"]; }

    [[deprecated("Use top-level vehicles[] mission layout instead of root form.")]]
    const ConfigNode& form() const { return config_["form"]; }

    const ConfigNode& environment() const { return config_["environment"]; }

    [[deprecated("Use top-level vehicles[] mission layout instead of root vehicle.")]]
    const ConfigNode& vehicle() const { return config_["vehicle"]; }

    [[deprecated("Use vehicles[].interaction instead of root interaction.")]]
    const ConfigNode& interaction() const { return config_["interaction"]; }

    const ConfigNode& outputs() const { return config_["outputs"]; }

    /// 获取组件配置列表
    [[deprecated("Use vehicles[].form/common/input/process/output/interaction components.")]]
    const ConfigNode& components() const { return config_["components"]; }

    /// 获取实体配置列表
    [[deprecated("Legacy entities[] missions are unsupported; use vehicles[].")]]
    const ConfigNode& entities() const { return config_["entities"]; }
    
    /// 获取指定组件的配置
    [[deprecated("Legacy component lookup is unsupported for vehicles[] missions.")]]
    const ConfigNode& componentConfig(const std::string& name) const {
        const auto& entity_list = config_["entities"];
        if (entity_list.isArray()) {
            for (size_t entity_index = 0; entity_index < entity_list.size(); ++entity_index) {
                const auto& comps = entity_list[entity_index]["components"];
                for (size_t i = 0; i < comps.size(); ++i) {
                    if (comps[i]["name"].asString() == name) {
                        return comps[i]["config"];
                    }
                }
            }
        }

        const auto& comps = config_["components"];
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

    bool hasEntities() const { return config_.has("entities"); }

    bool hasModernMissionLayout() const {
        return config_.has("vehicles");
    }
    
    /// 检查是否为多飞行器模式
    bool isMultiVehicle() const { return config_.has("vehicles"); }
    
private:
    std::unique_ptr<IConfigParser> parser_;
    ConfigNode config_;
};

} // namespace gnc::core
