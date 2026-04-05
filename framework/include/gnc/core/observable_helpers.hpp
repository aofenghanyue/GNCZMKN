/**
 * @file observable_helpers.hpp
 * @brief IObservable 实现辅助工具
 */
#pragma once

#include "gnc/common/math_types.hpp"
#include "gnc/interfaces/infrastructure/i_observable.hpp"

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

    void addVector3d(const std::string& name,
                     std::function<const Vector3d&()> getter) {
        fields_.push_back({name + ".x", [getter]() { return getter().x; }});
        fields_.push_back({name + ".y", [getter]() { return getter().y; }});
        fields_.push_back({name + ".z", [getter]() { return getter().z; }});
    }

    void addQuaterniond(const std::string& name,
                        std::function<const Quaterniond&()> getter) {
        fields_.push_back({name + ".w", [getter]() { return getter().w; }});
        fields_.push_back({name + ".x", [getter]() { return getter().x; }});
        fields_.push_back({name + ".y", [getter]() { return getter().y; }});
        fields_.push_back({name + ".z", [getter]() { return getter().z; }});
    }

    Fields build() const {
        return fields_;
    }

private:
    Fields fields_;
};

} // namespace gnc::core
