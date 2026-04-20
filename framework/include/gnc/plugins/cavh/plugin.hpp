#pragma once

#include "gnc/core/component_factory.hpp"
#include "gnc/core/plugin_registry.hpp"
#include "gnc/plugins/cavh/components/aero_table.hpp"
#include "gnc/plugins/cavh/components/constant_mass.hpp"

namespace gnc::plugins::cavh {

class CavhPlugin final : public gnc::core::Plugin {
public:
    const char* name() const override { return "cavh"; }
    gnc::core::PluginLayer layer() const override {
        return gnc::core::PluginLayer::Subsystem;
    }

    std::vector<std::string> dependencies() const override {
        return {"aero", "mass"};
    }

    void install(gnc::core::PluginRegistry&) const override {
        auto& factory = gnc::core::ComponentFactory::instance();

        factory.registerType<ConstantMass,
                             gnc::plugins::mass::IConstantMass,
                             gnc::interfaces::IObservable>(
            "cavh.constant_mass",
            gnc::core::ComponentCategory::Builtin,
            __FILE__);
        factory.registerType<AeroTable,
                             gnc::plugins::aero::IAeroModel,
                             gnc::interfaces::IObservable>(
            "cavh.aero_table",
            gnc::core::ComponentCategory::Builtin,
            __FILE__);
    }
};

GNC_REGISTER_PLUGIN(CavhPlugin)

} // namespace gnc::plugins::cavh
