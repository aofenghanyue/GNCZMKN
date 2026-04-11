#pragma once

#include "gnc/core/plugin_registry.hpp"
#include "gnc/plugins/environment/components/wgs84_earth.hpp"
#include "gnc/plugins/environment/components/standard_atmosphere.hpp"
#include "gnc/plugins/environment/components/spherical_gravity.hpp"

namespace gnc::plugins::environment {

class EnvironmentPlugin final : public gnc::core::Plugin {
public:
    const char* name() const override { return "environment"; }
    void install(gnc::core::PluginRegistry&) const override {}
};

GNC_REGISTER_PLUGIN(EnvironmentPlugin)

} // namespace gnc::plugins::environment
