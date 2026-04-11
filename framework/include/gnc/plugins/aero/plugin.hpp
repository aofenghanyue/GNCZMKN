#pragma once

#include "gnc/core/plugin_registry.hpp"
#include "gnc/plugins/aero/components/simple_polynomial_aero.hpp"

namespace gnc::plugins::aero {

class AeroPlugin final : public gnc::core::Plugin {
public:
    const char* name() const override { return "aero"; }
    void install(gnc::core::PluginRegistry&) const override {}
};

GNC_REGISTER_PLUGIN(AeroPlugin)

} // namespace gnc::plugins::aero
