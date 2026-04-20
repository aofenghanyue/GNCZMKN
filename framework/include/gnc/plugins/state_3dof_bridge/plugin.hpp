#pragma once

#include "gnc/core/component_factory.hpp"
#include "gnc/core/plugin_registry.hpp"
#include "gnc/plugins/state_3dof_bridge/components/force_to_local_acceleration_soviet.hpp"

namespace gnc::plugins::state_3dof_bridge {

class State3DoFBridgePlugin final : public gnc::core::Plugin {
public:
    const char* name() const override { return "state_3dof_bridge"; }
    gnc::core::PluginLayer layer() const override {
        return gnc::core::PluginLayer::System;
    }

    std::vector<std::string> dependencies() const override {
        return {"environment", "aero", "mass", "state_3dof"};
    }

    void install(gnc::core::PluginRegistry&) const override {
        auto& factory = gnc::core::ComponentFactory::instance();

        factory.registerType<ForceToLocalAccelerationSoviet,
                             gnc::plugins::state_3dof::IAccelerationProvider3DOF>(
            "state_3dof_bridge.force_to_local_acceleration_soviet",
            gnc::core::ComponentCategory::Builtin,
            __FILE__);
    }
};

GNC_REGISTER_PLUGIN(State3DoFBridgePlugin)

} // namespace gnc::plugins::state_3dof_bridge
