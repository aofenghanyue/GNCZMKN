#include <iostream>
#include <chrono>
#include <vector>
#include "gnc/common/math/calculus.hpp"
#include <Eigen/Dense>

using namespace gnc::math;

// Simple harmonic oscillator: dy/dt = [v; -k/m * x]
// Size 6 to simulate a more realistic state (e.g. 3D pos + 3D vel)
VectorX harmonic_oscillator(double t, const VectorX& y) {
    (void)t; // Unused
    double k = 1.0;
    double m = 1.0;
    int n = y.size();
    VectorX dydt(n);

    // Split into pos and vel
    int n_half = n / 2;
    for(int i=0; i<n_half; ++i) {
        dydt(i) = y(i + n_half);
        dydt(i + n_half) = -k/m * y(i);
    }
    return dydt;
}

int main() {
    int dim = 12; // 6DOF state
    VectorX y0 = VectorX::Zero(dim);
    y0(0) = 1.0; // Initial x position

    double t0 = 0.0;
    double h = 0.01;

    int iterations = 10000;

    std::cout << "Benchmarking RK45 with state dimension " << dim << " for " << iterations << " iterations." << std::endl;

    // --- Baseline ---
    // Warmup
    for(int i=0; i<100; ++i) {
        rk45_step(harmonic_oscillator, t0, y0, h);
    }

    auto start = std::chrono::high_resolution_clock::now();

    VectorX y = y0;
    double t = t0;
    for (int i = 0; i < iterations; ++i) {
        auto result = rk45_step(harmonic_oscillator, t, y, h);
        y = result.y_new;
        t += h;
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;

    std::cout << "Baseline RK45: " << iterations << " steps in " << elapsed.count() << "s" << std::endl;
    std::cout << "Average time per step: " << (elapsed.count() / iterations) * 1e6 << " us" << std::endl;
    std::cout << "Final state norm: " << y.norm() << std::endl;
    double baseline_norm = y.norm();

    // --- Optimized ---

    Rk45Workspace ws;
    ws.resize(dim);

    // Warmup
    for(int i=0; i<100; ++i) {
        rk45_step(harmonic_oscillator, t0, y0, h, ws);
    }

    start = std::chrono::high_resolution_clock::now();

    y = y0;
    t = t0;
    for (int i = 0; i < iterations; ++i) {
        auto result = rk45_step(harmonic_oscillator, t, y, h, ws);
        y = result.y_new;
        t += h;
    }

    end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_opt = end - start;

    std::cout << "Optimized RK45: " << iterations << " steps in " << elapsed_opt.count() << "s" << std::endl;
    std::cout << "Average time per step: " << (elapsed_opt.count() / iterations) * 1e6 << " us" << std::endl;
    std::cout << "Final state norm: " << y.norm() << std::endl;

    std::cout << "Speedup: " << elapsed.count() / elapsed_opt.count() << "x" << std::endl;

    if (std::abs(y.norm() - baseline_norm) > 1e-9) {
        std::cerr << "Verification FAILED: Results differ!" << std::endl;
        return 1;
    }

    return 0;
}
