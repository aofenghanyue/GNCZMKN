#pragma once

#include <algorithm>
#include <cctype>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace gnc::simflow {

struct SimFlowCaseValue {
    enum class Type {
        Number,
        String
    };

    Type type = Type::String;
    double number = 0.0;
    std::string string;
};

struct SimFlowCaseVariables {
    std::string case_id;
    std::unordered_map<std::string, SimFlowCaseValue> values;
};

inline std::string trim(const std::string& text) {
    size_t begin = 0;
    while (begin < text.size() &&
           std::isspace(static_cast<unsigned char>(text[begin]))) {
        ++begin;
    }
    size_t end = text.size();
    while (end > begin &&
           std::isspace(static_cast<unsigned char>(text[end - 1]))) {
        --end;
    }
    return text.substr(begin, end - begin);
}

inline std::vector<std::string> splitCommaLine(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
        fields.push_back(trim(field));
    }
    return fields;
}

inline SimFlowCaseValue parseCaseValue(const std::string& cell) {
    SimFlowCaseValue value;
    try {
        size_t consumed = 0;
        const double number = std::stod(cell, &consumed);
        while (consumed < cell.size() &&
               std::isspace(static_cast<unsigned char>(cell[consumed]))) {
            ++consumed;
        }
        if (consumed == cell.size()) {
            value.type = SimFlowCaseValue::Type::Number;
            value.number = number;
            return value;
        }
    } catch (const std::exception&) {
    }
    value.type = SimFlowCaseValue::Type::String;
    value.string = cell;
    return value;
}

inline std::vector<SimFlowCaseVariables> parseCsvCaseSource(
    const std::string& text,
    const std::vector<size_t>& rows) {
    std::stringstream stream(text);
    std::string line;
    if (!std::getline(stream, line)) {
        throw std::runtime_error("simflow CSV case source is empty.");
    }
    const auto header = splitCommaLine(line);
    if (header.size() < 2) {
        throw std::runtime_error(
            "simflow CSV case source must contain case_id and at least one input column.");
    }
    if (header[0] != "case_id") {
        throw std::runtime_error("simflow CSV case source first column must be case_id.");
    }

    std::vector<std::vector<std::string>> body;
    while (std::getline(stream, line)) {
        if (!trim(line).empty()) {
            body.push_back(splitCommaLine(line));
        }
    }

    std::vector<SimFlowCaseVariables> result;
    for (size_t row_index : rows) {
        if (row_index >= body.size()) {
            throw std::runtime_error("simflow CSV row index " +
                                     std::to_string(row_index) +
                                     " is out of range.");
        }
        const auto& row = body[row_index];
        if (row.size() != header.size()) {
            throw std::runtime_error("simflow CSV row " +
                                     std::to_string(row_index) +
                                     " column count does not match header.");
        }

        SimFlowCaseVariables current;
        current.case_id = row[0].empty() ? "case_" + std::to_string(row_index + 1)
                                         : row[0];
        for (size_t column = 1; column < header.size(); ++column) {
            current.values[header[column]] = parseCaseValue(row[column]);
        }
        result.push_back(std::move(current));
    }
    return result;
}

} // namespace gnc::simflow
