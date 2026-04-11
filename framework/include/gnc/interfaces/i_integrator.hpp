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

    virtual void step(const DerivativeFunc& f,
                      double t,
                      Eigen::VectorXd& x,
                      double dt) = 0;
    virtual const char* name() const = 0;
};

} // namespace gnc::interfaces
