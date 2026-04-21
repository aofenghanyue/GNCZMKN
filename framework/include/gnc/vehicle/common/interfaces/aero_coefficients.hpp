#pragma once

namespace gnc::vehicle::common {

struct AeroCoefficients {
    double lift_coefficient = 0.0;
    double drag_coefficient = 0.0;
    double side_force_coefficient = 0.0;
    double rolling_moment_coefficient = 0.0;
    double pitching_moment_coefficient = 0.0;
    double yawing_moment_coefficient = 0.0;
};

} // namespace gnc::vehicle::common
