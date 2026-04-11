#pragma once

#include "gnc/interfaces/i_integrator.hpp"

namespace gnc::core {

class RK4Integrator : public interfaces::IIntegrator {
public:
    void step(const DerivativeFunc& f,
              double t,
              Eigen::VectorXd& x,
              double dt) override {
        Eigen::VectorXd k1(x.size());
        Eigen::VectorXd k2(x.size());
        Eigen::VectorXd k3(x.size());
        Eigen::VectorXd k4(x.size());
        Eigen::VectorXd temp(x.size());

        f(t, x, k1);

        temp = x + 0.5 * dt * k1;
        f(t + 0.5 * dt, temp, k2);

        temp = x + 0.5 * dt * k2;
        f(t + 0.5 * dt, temp, k3);

        temp = x + dt * k3;
        f(t + dt, temp, k4);

        x += (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    }

    const char* name() const override {
        return "rk4";
    }
};

} // namespace gnc::core
