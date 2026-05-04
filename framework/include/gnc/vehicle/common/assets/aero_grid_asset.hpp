#pragma once

#include "gnc/core/config_manager.hpp"
#include "gnc/core/config_reader.hpp"

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace gnc::vehicle::common::assets {

struct AeroGridReference {
    double area_m2 = 1.0;
    double length_m = 1.0;
    double span_m = 1.0;
    double moment_ref_x_m = 0.0;
};

struct AeroGridAxis {
    std::string name;
    std::vector<double> nodes;
};

struct AeroGridColumn {
    std::string name;
};

struct AeroGridLayout {
    std::vector<std::string> axis_order;
    std::string fastest_varying_axis;
};

struct AeroGridAsset {
    std::string schema;
    AeroGridReference reference;
    std::vector<AeroGridAxis> axes;
    AeroGridLayout layout;
    std::vector<AeroGridColumn> columns;
    std::vector<std::vector<double>> data;

    bool hasColumn(const std::string& name) const {
        return std::any_of(columns.begin(),
                           columns.end(),
                           [&](const AeroGridColumn& column) {
                               return column.name == name;
                           });
    }

    size_t columnIndex(const std::string& name) const {
        for (size_t i = 0; i < columns.size(); ++i) {
            if (columns[i].name == name) {
                return i;
            }
        }
        throw std::runtime_error("Aero grid column '" + name + "' was not found.");
    }
};

inline bool isStrictlyIncreasing(const std::vector<double>& values) {
    for (size_t i = 1; i < values.size(); ++i) {
        if (!(values[i] > values[i - 1])) {
            return false;
        }
    }
    return true;
}

inline std::vector<double> readRequiredNumberArray(
    const gnc::core::ConfigNode& node,
    const std::string& path) {
    if (!node.isArray()) {
        throw std::runtime_error(path + " must be an array.");
    }
    std::vector<double> values;
    values.reserve(node.size());
    for (size_t i = 0; i < node.size(); ++i) {
        if (!node[i].isNumber()) {
            throw std::runtime_error(path + "[" + std::to_string(i) +
                                     "] must be a number.");
        }
        values.push_back(node[i].asDouble());
    }
    return values;
}

inline AeroGridReference parseAeroGridReference(
    const gnc::core::ConfigNode& node,
    const std::string& source_description) {
    auto reader = gnc::core::ConfigReader(node, source_description + ".reference");
    AeroGridReference reference;
    reference.area_m2 = reader.requiredDouble("area_m2");
    reference.length_m = reader.requiredDouble("length_m");
    reference.span_m = reader.optionalDouble("span_m", 1.0);
    reference.moment_ref_x_m = reader.optionalDouble("moment_ref_x_m", 0.0);
    if (reference.area_m2 <= 0.0) {
        throw std::runtime_error(source_description +
                                 ".reference.area_m2 must be positive.");
    }
    if (reference.length_m <= 0.0) {
        throw std::runtime_error(source_description +
                                 ".reference.length_m must be positive.");
    }
    if (reference.span_m <= 0.0) {
        throw std::runtime_error(source_description +
                                 ".reference.span_m must be positive.");
    }
    return reference;
}

inline std::vector<AeroGridAxis> parseAeroGridAxes(
    const gnc::core::ConfigNode& node,
    const std::string& source_description) {
    if (!node.isArray() || node.size() == 0) {
        throw std::runtime_error(source_description +
                                 ".axes must be a non-empty array.");
    }

    std::unordered_set<std::string> names;
    std::vector<AeroGridAxis> axes;
    axes.reserve(node.size());
    for (size_t i = 0; i < node.size(); ++i) {
        const std::string path =
            source_description + ".axes[" + std::to_string(i) + "]";
        if (!node[i].isObject()) {
            throw std::runtime_error(path + " must be an object.");
        }
        gnc::core::ConfigReader reader(node[i], path);
        AeroGridAxis axis;
        axis.name = reader.requiredString("name");
        if (!names.insert(axis.name).second) {
            throw std::runtime_error(path + ".name duplicates axis '" +
                                     axis.name + "'.");
        }
        axis.nodes = reader.requiredDoubleArray("nodes");
        if (axis.nodes.size() < 2) {
            throw std::runtime_error(path + ".nodes must contain at least 2 numbers.");
        }
        if (!isStrictlyIncreasing(axis.nodes)) {
            throw std::runtime_error(path + ".nodes must be strictly increasing.");
        }
        axes.push_back(std::move(axis));
    }
    return axes;
}

inline AeroGridLayout parseAeroGridLayout(
    const gnc::core::ConfigNode& node,
    const std::vector<AeroGridAxis>& axes,
    const std::string& source_description) {
    AeroGridLayout layout;
    layout.axis_order.reserve(axes.size());
    for (const auto& axis : axes) {
        layout.axis_order.push_back(axis.name);
    }
    layout.fastest_varying_axis = layout.axis_order.back();

    if (node.isNull()) {
        return layout;
    }
    if (!node.isObject()) {
        throw std::runtime_error(source_description + ".layout must be an object.");
    }

    const std::string path = source_description + ".layout";
    if (node["axis_order"].isArray()) {
        layout.axis_order.clear();
        for (size_t i = 0; i < node["axis_order"].size(); ++i) {
            if (!node["axis_order"][i].isString()) {
                throw std::runtime_error(path + ".axis_order[" +
                                         std::to_string(i) +
                                         "] must be a string.");
            }
            layout.axis_order.push_back(node["axis_order"][i].asString());
        }
    }
    if (node["fastest_varying_axis"].isString()) {
        layout.fastest_varying_axis = node["fastest_varying_axis"].asString();
    }

    std::unordered_set<std::string> axis_names;
    for (const auto& axis : axes) {
        axis_names.insert(axis.name);
    }
    if (layout.axis_order.size() != axes.size()) {
        throw std::runtime_error(path + ".axis_order must list every axis once.");
    }
    std::unordered_set<std::string> seen;
    for (const auto& name : layout.axis_order) {
        if (axis_names.count(name) == 0) {
            throw std::runtime_error(path + ".axis_order references unknown axis '" +
                                     name + "'.");
        }
        if (!seen.insert(name).second) {
            throw std::runtime_error(path + ".axis_order duplicates axis '" +
                                     name + "'.");
        }
    }
    if (axis_names.count(layout.fastest_varying_axis) == 0) {
        throw std::runtime_error(path +
                                 ".fastest_varying_axis references unknown axis '" +
                                 layout.fastest_varying_axis + "'.");
    }
    return layout;
}

inline std::vector<AeroGridColumn> parseAeroGridColumns(
    const gnc::core::ConfigNode& node,
    const std::string& source_description) {
    if (!node.isArray() || node.size() == 0) {
        throw std::runtime_error(source_description +
                                 ".columns must be a non-empty array.");
    }

    std::unordered_set<std::string> names;
    std::vector<AeroGridColumn> columns;
    columns.reserve(node.size());
    for (size_t i = 0; i < node.size(); ++i) {
        const std::string path =
            source_description + ".columns[" + std::to_string(i) + "]";
        AeroGridColumn column;
        if (node[i].isString()) {
            column.name = node[i].asString();
        } else if (node[i].isObject()) {
            column.name = gnc::core::ConfigReader(node[i], path).requiredString("name");
        } else {
            throw std::runtime_error(path + " must be a string or object.");
        }
        if (column.name.empty()) {
            throw std::runtime_error(path + ".name must be non-empty.");
        }
        if (!names.insert(column.name).second) {
            throw std::runtime_error(path + " duplicates column '" + column.name + "'.");
        }
        columns.push_back(std::move(column));
    }
    return columns;
}

inline std::vector<std::vector<double>> parseAeroGridData(
    const gnc::core::ConfigNode& node,
    size_t expected_rows,
    size_t expected_columns,
    const std::string& source_description) {
    if (!node.isArray()) {
        throw std::runtime_error(source_description + ".data must be an array.");
    }
    if (node.size() != expected_rows) {
        throw std::runtime_error(source_description + ".data row count must be " +
                                 std::to_string(expected_rows) + " but got " +
                                 std::to_string(node.size()) + ".");
    }

    std::vector<std::vector<double>> data;
    data.reserve(node.size());
    for (size_t row = 0; row < node.size(); ++row) {
        const std::string row_path =
            source_description + ".data[" + std::to_string(row) + "]";
        if (!node[row].isArray() || node[row].size() != expected_columns) {
            throw std::runtime_error(row_path + " must contain exactly " +
                                     std::to_string(expected_columns) + " numbers.");
        }
        data.push_back(readRequiredNumberArray(node[row], row_path));
    }
    return data;
}

inline size_t expectedAeroGridRows(const std::vector<AeroGridAxis>& axes) {
    size_t rows = 1;
    for (const auto& axis : axes) {
        rows *= axis.nodes.size();
    }
    return rows;
}

inline AeroGridAsset parseAeroGridAsset(
    const gnc::core::ConfigNode& node,
    const std::string& source_description = "aero grid asset") {
    if (!node.isObject()) {
        throw std::runtime_error(source_description + " must be an object.");
    }

    gnc::core::ConfigReader reader(node, source_description);
    AeroGridAsset asset;
    asset.schema = reader.requiredString("schema");
    if (asset.schema != "gnc.aero.grid.v1") {
        throw std::runtime_error(source_description +
                                 ".schema must be 'gnc.aero.grid.v1'.");
    }
    asset.reference = parseAeroGridReference(node["reference"], source_description);
    asset.axes = parseAeroGridAxes(node["axes"], source_description);
    asset.layout = parseAeroGridLayout(node["layout"], asset.axes, source_description);
    asset.columns = parseAeroGridColumns(node["columns"], source_description);
    asset.data = parseAeroGridData(node["data"],
                                   expectedAeroGridRows(asset.axes),
                                   asset.columns.size(),
                                   source_description);
    return asset;
}

} // namespace gnc::vehicle::common::assets
