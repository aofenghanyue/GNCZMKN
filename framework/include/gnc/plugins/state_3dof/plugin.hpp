#pragma once

#include "gnc/core/plugin_registry.hpp"
#include "gnc/plugins/state_3dof/components/point_mass_cartesian.hpp"
#include "gnc/plugins/state_3dof/components/point_mass_spherical.hpp"

namespace gnc::plugins::state_3dof {

class State3DoFPlugin final : public gnc::core::Plugin {
public:
    const char* name() const override { return "state_3dof"; }
    void install(gnc::core::PluginRegistry&) const override {}
};

GNC_REGISTER_PLUGIN(State3DoFPlugin)

} // namespace gnc::plugins::state_3dof
