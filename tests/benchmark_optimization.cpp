#include "gnc/common/math/optimization.hpp"
#include <iostream>
#include <chrono>
#include <vector>

using namespace gnc::math;

// Rosenbrock function
// f(x, y) = (a - x)^2 + b * (y - x^2)^2
// with a = 1, b = 100
// Minimum at (1, 1)
double rosenbrock(const VectorX& x) {
    double a = 1.0;
    double b = 100.0;
    return std::pow(a - x(0), 2) + b * std::pow(x(1) - std::pow(x(0), 2), 2);
}

int main() {
    std::cout << "Starting optimization benchmark (Nelder-Mead)..." << std::endl;

    VectorX x0(2);
    x0 << 0.0, 0.0; // Start at (0,0)

    // Warmup
    nelder_mead(rosenbrock, x0, 1e-6, 1000);

    const int num_runs = 5000;
    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < num_runs; ++i) {
        OptResult result = nelder_mead(rosenbrock, x0, 1e-8, 1000);
        // Basic check to ensure it ran
        if (result.f_opt > 1e-4) {
             std::cerr << "Warning: Optimization failed to converge on run " << i << std::endl;
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end - start;

    double total_time = diff.count();
    double avg_time = total_time / num_runs;

    std::cout << "Total time for " << num_runs << " runs: " << total_time << " s" << std::endl;
    std::cout << "Average time per run: " << avg_time * 1e6 << " us" << std::endl;

    // Verify correctness
    OptResult final_result = nelder_mead(rosenbrock, x0, 1e-8, 1000);
    std::cout << "Final Result: x = [" << final_result.x_opt.transpose() << "], f = " << final_result.f_opt << std::endl;

    if (std::abs(final_result.x_opt(0) - 1.0) < 5e-4 && std::abs(final_result.x_opt(1) - 1.0) < 5e-4) {
        std::cout << "Verification: SUCCESS" << std::endl;
    } else {
        std::cout << "Verification: FAILURE" << std::endl;
    }

    return 0;
}
