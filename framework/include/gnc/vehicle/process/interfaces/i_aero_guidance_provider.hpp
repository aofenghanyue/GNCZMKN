#pragma once

#include "gnc/vehicle/process/interfaces/aero_guidance_command.hpp"

namespace gnc::vehicle::process {

class IAeroGuidanceProvider {
public:
    virtual ~IAeroGuidanceProvider() = default;

    virtual const AeroGuidanceCommand& getAeroGuidanceCommand() const = 0;
    virtual bool isGuidanceActive() const = 0;
};

} // namespace gnc::vehicle::process
