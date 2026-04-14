#pragma once

#include "gnc/core/component_factory.hpp"
#include "gnc/core/plugin_registry.hpp"
#include "gnc/plugins/environment/components/wgs84_earth.hpp"
#include "gnc/plugins/environment/components/standard_atmosphere.hpp"
#include "gnc/plugins/environment/components/spherical_gravity.hpp"

namespace gnc::plugins::environment {

class EnvironmentPlugin final : public gnc::core::Plugin {
public:
    const char* name() const override { return "environment"; }
    gnc::core::PluginLayer layer() const override {
        return gnc::core::PluginLayer::Subsystem;
    }

    void install(gnc::core::PluginRegistry&) const override {
        auto& factory = gnc::core::ComponentFactory::instance();

        factory.registerType<Wgs84Earth, IEarth>(
            "environment.wgs84_earth",
            gnc::core::ComponentCategory::Builtin,
            __FILE__);
        factory.registerType<StandardAtmosphere, IAtmosphere>(
            "environment.standard_atmosphere",
            gnc::core::ComponentCategory::Builtin,
            __FILE__);
        factory.registerType<SphericalGravity, IGravity>(
            "environment.spherical_gravity",
            gnc::core::ComponentCategory::Builtin,
            __FILE__);
    }
};

GNC_REGISTER_PLUGIN(EnvironmentPlugin)

} // namespace gnc::plugins::environment
