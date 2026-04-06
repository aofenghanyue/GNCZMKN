#pragma once

#include "gnc/core/state_layout.hpp"
#include "gnc/common/math/eigen_types.hpp"

#include <string>

namespace gnc::interfaces {

class IDynamicsModel {
public:
    virtual ~IDynamicsModel() = default;

    virtual const gnc::core::StateLayout& getStateLayout() const = 0;
    virtual void computeDerivatives(double t,
                                    const Eigen::VectorXd& x,
                                    Eigen::VectorXd& dxdt) const = 0;
    virtual const Eigen::VectorXd& getState() const = 0;
    virtual void setState(const Eigen::VectorXd& x) = 0;
    virtual Eigen::VectorXd getInitialState() const = 0;

    double getStateValue(const std::string& name) const {
        const int idx = getStateLayout().indexOf(name);
        const auto& state = getState();
        return (idx >= 0 && idx < state.size()) ? state[idx] : 0.0;
    }
};

} // namespace gnc::interfaces
