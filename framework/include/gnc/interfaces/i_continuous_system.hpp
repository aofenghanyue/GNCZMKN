#pragma once

#include "gnc/core/state_layout.hpp"
#include "gnc/common/math/eigen_types.hpp"

#include <string>

namespace gnc::interfaces {

// Contract for state components advanced by the simulation integrator.
class IContinuousSystem {
public:
    virtual ~IContinuousSystem() = default;

    virtual const gnc::core::StateLayout& getStateLayout() const = 0;
    virtual void computeDerivatives(double time,
                                    const Eigen::VectorXd& state,
                                    Eigen::VectorXd& derivative) const = 0;
    virtual const Eigen::VectorXd& getState() const = 0;
    virtual void setState(const Eigen::VectorXd& state) = 0;
    virtual Eigen::VectorXd getInitialState() const = 0;

    double getStateValue(const std::string& name) const {
        const int index = getStateLayout().indexOf(name);
        const auto& state = getState();
        return (index >= 0 && index < state.size()) ? state[index] : 0.0;
    }
};

} // namespace gnc::interfaces
