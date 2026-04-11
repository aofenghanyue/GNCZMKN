#pragma once

#include "gnc/interfaces/i_integrator.hpp"

namespace gnc::core {

class EulerIntegrator : public interfaces::IIntegrator {
public:
    void step(const DerivativeFunc& f,
              double t,
              Eigen::VectorXd& x,
              double dt) override {
        Eigen::VectorXd dxdt(x.size());
        f(t, x, dxdt);
        x += dt * dxdt;
    }

    const char* name() const override {
        return "euler";
    }
};

} // namespace gnc::core
