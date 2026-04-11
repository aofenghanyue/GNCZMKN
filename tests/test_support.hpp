#pragma once

#include "gnc/common/math/eigen_types.hpp"
#include "gnc/core/config_manager.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace test_support {

inline void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

inline void requireNear(double actual,
                        double expected,
                        double tolerance,
                        const std::string& message) {
    if (std::abs(actual - expected) > tolerance) {
        std::ostringstream oss;
        oss << message << " (actual=" << actual
            << ", expected=" << expected
            << ", tol=" << tolerance << ")";
        throw std::runtime_error(oss.str());
    }
}

inline void requireVectorNear(const gnc::math::Vector3& actual,
                              const gnc::math::Vector3& expected,
                              double tolerance,
                              const std::string& message) {
    if (!actual.isApprox(expected, tolerance)) {
        std::ostringstream oss;
        oss << message << " (actual=" << actual.transpose()
            << ", expected=" << expected.transpose()
            << ", tol=" << tolerance << ")";
        throw std::runtime_error(oss.str());
    }
}

inline gnc::core::ConfigNode number(double value) {
    return gnc::core::ConfigNode::makeNumber(value);
}

inline gnc::core::ConfigNode string(const std::string& value) {
    return gnc::core::ConfigNode::makeString(value);
}

inline gnc::core::ConfigNode boolean(bool value) {
    return gnc::core::ConfigNode::makeBool(value);
}

inline std::pair<std::string, gnc::core::ConfigNode> field(
    std::string key,
    gnc::core::ConfigNode value) {
    return {std::move(key), std::move(value)};
}

inline gnc::core::ConfigNode object(
    std::initializer_list<std::pair<std::string, gnc::core::ConfigNode>> fields) {
    auto node = gnc::core::ConfigNode::makeObject();
    for (const auto& [key, value] : fields) {
        node.set(key, value);
    }
    return node;
}

inline gnc::core::ConfigNode array(
    std::initializer_list<gnc::core::ConfigNode> values) {
    auto node = gnc::core::ConfigNode::makeArray();
    for (const auto& value : values) {
        node.push(value);
    }
    return node;
}

} // namespace test_support
