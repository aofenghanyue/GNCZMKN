/**
 * @file i_observable.hpp
 * @brief 组件可观测性接口
 */
#pragma once

#include <functional>
#include <string>
#include <vector>

namespace gnc::interfaces {

struct ObservableField {
    std::string name;
    std::function<double()> getter;
};

class IObservable {
public:
    virtual ~IObservable() = default;
    virtual std::vector<ObservableField> getObservableFields() const = 0;
};

} // namespace gnc::interfaces
