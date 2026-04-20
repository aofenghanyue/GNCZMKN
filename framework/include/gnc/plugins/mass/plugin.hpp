#pragma once

#include "gnc/core/component_factory.hpp"
#include "gnc/core/plugin_registry.hpp"
#include "gnc/plugins/mass/components/continuous_constant_rate_mass.hpp"

namespace gnc::plugins::mass {

class MassPlugin final : public gnc::core::Plugin {
public:
    const char* name() const override { return "mass"; }
    gnc::core::PluginLayer layer() const override {
        return gnc::core::PluginLayer::Subsystem;
    }

    void install(gnc::core::PluginRegistry&) const override {
        auto& factory = gnc::core::ComponentFactory::instance();

        factory.registerType<ContinuousConstantRateMass,
                             gnc::interfaces::IContinuousSystem,
                             IContinuousMass,
                             gnc::interfaces::IObservable>(
            "mass.continuous_constant_rate",
            gnc::core::ComponentCategory::Builtin,
            __FILE__);
    }
};

GNC_REGISTER_PLUGIN(MassPlugin)

} // namespace gnc::plugins::mass
