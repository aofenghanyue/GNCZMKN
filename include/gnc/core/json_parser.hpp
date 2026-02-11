/**
 * @file json_parser.hpp
 * @brief 简单 JSON 解析器
 */
#pragma once

#include "config_node.hpp"
#include "gnc/common/logger.hpp"
#include <string>
#include <fstream>
#include <sstream>
#include <cctype>

namespace gnc::core {

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

} // namespace gnc::core
