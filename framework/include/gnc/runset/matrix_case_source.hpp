#pragma once

#include <cctype>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace gnc::runset {

struct MatrixCase {
    std::unordered_map<std::string, double> inputs;
};

inline std::vector<std::string> splitCommaLine(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
        fields.push_back(field);
    }
    return fields;
}

inline double parseNumericMatrixCell(const std::string& cell,
                                     size_t row_index,
                                     const std::string& column_name) {
    try {
        size_t consumed = 0;
        const double value = std::stod(cell, &consumed);
        while (consumed < cell.size() &&
               std::isspace(static_cast<unsigned char>(cell[consumed]))) {
            ++consumed;
        }
        if (consumed != cell.size()) {
            throw std::invalid_argument("trailing non-whitespace");
        }
        return value;
    } catch (const std::exception&) {
        throw std::runtime_error("matrix row " + std::to_string(row_index) +
                                 " column '" + column_name + "' must be numeric.");
    }
}

inline std::vector<MatrixCase> parseMatrixCases(const std::string& text,
                                                const std::vector<size_t>& rows) {
    std::stringstream stream(text);
    std::string line;
    if (!std::getline(stream, line)) {
        throw std::runtime_error("matrix case source is empty.");
    }
    const auto header = splitCommaLine(line);
    if (header.size() < 2) {
        throw std::runtime_error(
            "matrix case source must contain case_id and at least one input column.");
    }
    if (header[0] != "case_id") {
        throw std::runtime_error("matrix case source first column must be case_id.");
    }

    std::vector<std::vector<std::string>> body;
    while (std::getline(stream, line)) {
        if (!line.empty()) {
            body.push_back(splitCommaLine(line));
        }
    }

    std::vector<MatrixCase> result;
    for (size_t row_index : rows) {
        if (row_index >= body.size()) {
            throw std::runtime_error("matrix row index " + std::to_string(row_index) +
                                     " is out of range.");
        }
        const auto& row = body[row_index];
        if (row.size() != header.size()) {
            throw std::runtime_error("matrix row " + std::to_string(row_index) +
                                     " column count does not match header.");
        }

        MatrixCase current;
        for (size_t column = 1; column < header.size(); ++column) {
            current.inputs[header[column]] =
                parseNumericMatrixCell(row[column], row_index, header[column]);
        }
        result.push_back(std::move(current));
    }
    return result;
}

} // namespace gnc::runset
