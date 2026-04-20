#pragma once

// Builtin plugins are included in layer order so dependency checks in
// PluginRegistry::registerPlugin() see lower-level plugins first.

// Layer 1: hardware / subsystem plugins
#include "gnc/plugins/environment/plugin.hpp"
#include "gnc/plugins/aero/plugin.hpp"
#include "gnc/plugins/mass/plugin.hpp"
#include "gnc/plugins/cavh/plugin.hpp"
#include "gnc/plugins/state_3dof/plugin.hpp"

// Layer 2: system plugins
#include "gnc/plugins/state_3dof_bridge/plugin.hpp"
#include "gnc/plugins/flight_state_3dof/plugin.hpp"
#include "gnc/plugins/soviet_coord/plugin.hpp"
