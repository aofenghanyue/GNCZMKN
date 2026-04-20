#pragma once

#include "gnc/core/component_factory.hpp"
#include "gnc/core/plugin_registry.hpp"
#include "gnc/plugins/flight_state_3dof/components/soviet_observer.hpp"

namespace gnc::plugins::flight_state_3dof {

class FlightState3DoFPlugin final : public gnc::core::Plugin {
public:
    const char* name() const override { return "flight_state_3dof"; }
    gnc::core::PluginLayer layer() const override {
        return gnc::core::PluginLayer::System;
    }

    std::vector<std::string> dependencies() const override {
        return {"environment", "state_3dof"};
    }

    void install(gnc::core::PluginRegistry&) const override {
        auto& factory = gnc::core::ComponentFactory::instance();

        factory.registerType<SovietObserver,
                             IFlightState3DOFSovietObserver,
                             gnc::interfaces::IObservable>(
            "flight_state_3dof.soviet_observer",
            gnc::core::ComponentCategory::Builtin,
            __FILE__);
    }
};

GNC_REGISTER_PLUGIN(FlightState3DoFPlugin)

} // namespace gnc::plugins::flight_state_3dof
