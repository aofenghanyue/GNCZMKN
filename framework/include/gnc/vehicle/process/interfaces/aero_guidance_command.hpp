#pragma once

namespace gnc::vehicle::process {

struct AeroGuidanceCommand {
    double angle_of_attack_rad = 0.0;
    double bank_angle_rad = 0.0;
    double timestamp = 0.0;
};

} // namespace gnc::vehicle::process
