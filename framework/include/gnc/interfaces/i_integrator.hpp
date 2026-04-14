#pragma once

#include "gnc/common/math/eigen_types.hpp"

#include <functional>

namespace gnc::interfaces {

class IIntegrator {
public:
    virtual ~IIntegrator() = default;

    using DerivativeFunc = std::function<void(double t,
                                              const Eigen::VectorXd& x,
                                              Eigen::VectorXd& dxdt)>;

    /// Execute one fixed simulation step.
    /// @note The current framework assumes a fixed timestep (`simulation.dt`).
    ///       Supporting variable-step integrators requires a simulator-loop
    ///       refactor in addition to any interface change. See ADR-004.
    virtual void step(const DerivativeFunc& f,
                      double t,
                      Eigen::VectorXd& x,
                      double dt) = 0;
    virtual const char* name() const = 0;
};

} // namespace gnc::interfaces
