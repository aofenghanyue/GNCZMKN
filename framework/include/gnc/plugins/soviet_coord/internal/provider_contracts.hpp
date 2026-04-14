#pragma once

#include "gnc/plugins/state_3dof/interfaces/i_body_airspeed_provider.hpp"
#include "gnc/plugins/state_3dof/interfaces/i_body_attitude_provider.hpp"
#include "gnc/plugins/state_3dof/interfaces/i_velocity_direction_provider.hpp"

namespace gnc::plugins::soviet_coord {

using IBodyAttitudeProvider = gnc::plugins::state_3dof::IBodyAttitudeProvider;
using IVelocityDirectionProvider = gnc::plugins::state_3dof::IVelocityDirectionProvider;
using IBodyAirspeedProvider = gnc::plugins::state_3dof::IBodyAirspeedProvider;

} // namespace gnc::plugins::soviet_coord
