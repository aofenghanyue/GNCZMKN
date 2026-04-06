#include "gnc/core/integrators/euler_integrator.hpp"
#include "gnc/core/integrators/rk4_integrator.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

int main() {
    gnc::core::EulerIntegrator euler;
    gnc::core::RK4Integrator rk4;

    Eigen::VectorXd x0(1);
    x0[0] = 1.0;

    auto derivative = [](double, const Eigen::VectorXd& x, Eigen::VectorXd& dxdt) {
        dxdt.resize(1);
        dxdt[0] = -x[0];
    };

    Eigen::VectorXd x_euler = x0;
    Eigen::VectorXd x_rk4 = x0;

    const double dt = 0.1;
    for (int i = 0; i < 10; ++i) {
        euler.step(derivative, i * dt, x_euler, dt);
        rk4.step(derivative, i * dt, x_rk4, dt);
    }

    const double exact = std::exp(-1.0);
    const double euler_error = std::abs(x_euler[0] - exact);
    const double rk4_error = std::abs(x_rk4[0] - exact);

    std::cout << "Euler error: " << euler_error << "\n";
    std::cout << "RK4 error: " << rk4_error << "\n";

    if (rk4_error >= euler_error) {
        std::cerr << "RK4 should be more accurate than Euler\n";
        return 1;
    }

    if (rk4_error > 1e-4) {
        std::cerr << "RK4 accuracy is worse than expected\n";
        return 1;
    }

    return 0;
}
