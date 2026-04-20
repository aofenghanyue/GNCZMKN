#pragma once

#include "gnc/core/component_factory.hpp"
#include "gnc/core/plugin_registry.hpp"
#include "gnc/plugins/state_3dof/components/point_mass_cartesian.hpp"
#include "gnc/plugins/state_3dof/components/point_mass_spherical_soviet.hpp"

namespace gnc::plugins::state_3dof {

class State3DoFPlugin final : public gnc::core::Plugin {
public:
    const char* name() const override { return "state_3dof"; }
    gnc::core::PluginLayer layer() const override {
        return gnc::core::PluginLayer::Subsystem;
    }

    std::vector<std::string> dependencies() const override {
        return {"environment"};
    }

    void install(gnc::core::PluginRegistry&) const override {
        auto& factory = gnc::core::ComponentFactory::instance();

        factory.registerType<PointMassCartesian,
                             gnc::interfaces::IContinuousSystem,
                             IStateSolver3DOF,
                             gnc::interfaces::IObservable>(
            "state_3dof.point_mass_cartesian",
            gnc::core::ComponentCategory::Builtin,
            __FILE__);
        factory.registerType<PointMassSphericalSoviet,
                             gnc::interfaces::IContinuousSystem,
                             IStateSolver3DOF,
                             ISovietSphericalState3DOF,
                             IVelocityDirectionProvider,
                             gnc::interfaces::IObservable>(
            "state_3dof.point_mass_spherical_soviet",
            gnc::core::ComponentCategory::Builtin,
            __FILE__);
    }
};

GNC_REGISTER_PLUGIN(State3DoFPlugin)

} // namespace gnc::plugins::state_3dof
