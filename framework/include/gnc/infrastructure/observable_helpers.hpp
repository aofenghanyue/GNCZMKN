/**
 * @file observable_helpers.hpp
 * @brief IObservable 实现辅助工具
 */
#pragma once

#include "gnc/common/math/eigen_types.hpp"
#include "gnc/interfaces/i_observable.hpp"

#include <functional>
#include <string>
#include <vector>

namespace gnc::core {

class ObservableFieldBuilder {
public:
    using Fields = std::vector<interfaces::ObservableField>;

    void addScalar(const std::string& name, std::function<double()> getter) {
        fields_.push_back({name, std::move(getter)});
    }

    void addVector3(const std::string& name,
                    std::function<const gnc::math::Vector3&()> getter) {
        fields_.push_back({name + ".x", [getter]() { return getter().x(); }});
        fields_.push_back({name + ".y", [getter]() { return getter().y(); }});
        fields_.push_back({name + ".z", [getter]() { return getter().z(); }});
    }

    void addQuaternion(const std::string& name,
                       std::function<const Eigen::Quaterniond&()> getter) {
        fields_.push_back({name + ".w", [getter]() { return getter().w(); }});
        fields_.push_back({name + ".x", [getter]() { return getter().x(); }});
        fields_.push_back({name + ".y", [getter]() { return getter().y(); }});
        fields_.push_back({name + ".z", [getter]() { return getter().z(); }});
    }

    Fields build() const {
        return fields_;
    }

private:
    Fields fields_;
};

} // namespace gnc::core
