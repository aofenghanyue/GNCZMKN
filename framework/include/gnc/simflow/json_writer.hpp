#pragma once

#include "gnc/core/config_manager.hpp"

#include <algorithm>
#include <iomanip>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

namespace gnc::simflow {

inline std::string escapeJsonString(const std::string& value) {
    std::string result;
    for (char ch : value) {
        switch (ch) {
        case '\\':
            result += "\\\\";
            break;
        case '"':
            result += "\\\"";
            break;
        case '\n':
            result += "\\n";
            break;
        case '\t':
            result += "\\t";
            break;
        default:
            result += ch;
            break;
        }
    }
    return result;
}

inline void writeJsonNode(std::ostream& out,
                          const gnc::core::ConfigNode& node,
                          int indent) {
    const std::string pad(static_cast<size_t>(indent), ' ');
    if (node.isNull()) {
        out << "null";
    } else if (node.isBool()) {
        out << (node.asBool() ? "true" : "false");
    } else if (node.isNumber()) {
        out << std::setprecision(15) << node.asDouble();
    } else if (node.isString()) {
        out << '"' << escapeJsonString(node.asString()) << '"';
    } else if (node.isArray()) {
        out << "[";
        for (size_t i = 0; i < node.size(); ++i) {
            if (i > 0) {
                out << ", ";
            }
            writeJsonNode(out, node[i], indent);
        }
        out << "]";
    } else if (node.isObject()) {
        std::vector<std::string> keys;
        for (const auto& [key, value] : node) {
            (void)value;
            keys.push_back(key);
        }
        std::sort(keys.begin(), keys.end());
        out << "{\n";
        for (size_t i = 0; i < keys.size(); ++i) {
            out << pad << "  \"" << escapeJsonString(keys[i]) << "\": ";
            writeJsonNode(out, node[keys[i]], indent + 2);
            out << (i + 1 < keys.size() ? ",\n" : "\n");
        }
        out << pad << "}";
    }
}

inline std::string writeJson(const gnc::core::ConfigNode& node) {
    std::ostringstream out;
    writeJsonNode(out, node, 0);
    out << "\n";
    return out.str();
}

} // namespace gnc::simflow
