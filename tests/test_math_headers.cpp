// Simple compile test for new math headers
#include "gnc/common/math/special_functions.hpp"
#include "gnc/common/math/linalg.hpp"
#include "gnc/common/math/interp.hpp"
#include "gnc/common/math/calculus.hpp"
#include "gnc/common/math/roots.hpp"
#include "gnc/common/math/statistics.hpp"
#include "gnc/common/math/optimization.hpp"
#include "gnc/common/math/nonlinear.hpp"

int main() {
    // Test special functions
    double s = gnc::math::sat(1.5, -1.0, 1.0);
    double sg = gnc::math::sign(5.0);
    
    // Test pinv
    gnc::math::MatrixX A(2,3);
    A << 1, 2, 3, 4, 5, 6;
    auto Ap = gnc::math::pinv(A);
    
    return 0;
}
