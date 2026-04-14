#pragma once

#include "gnc/core/component_factory.hpp"
#include "gnc/core/plugin_registry.hpp"
#include "gnc/plugins/aero/components/simple_polynomial_aero.hpp"

namespace gnc::plugins::aero {

class AeroPlugin final : public gnc::core::Plugin {
public:
    const char* name() const override { return "aero"; }
    gnc::core::PluginLayer layer() const override {
        return gnc::core::PluginLayer::Subsystem;
    }

    void install(gnc::core::PluginRegistry&) const override {
        auto& factory = gnc::core::ComponentFactory::instance();

        factory.registerType<SimplePolynomialAero,
                             IAeroModel,
                             gnc::interfaces::IObservable>(
            "aero.simple_polynomial",
            gnc::core::ComponentCategory::Builtin,
            __FILE__);
    }
};

GNC_REGISTER_PLUGIN(AeroPlugin)

} // namespace gnc::plugins::aero
