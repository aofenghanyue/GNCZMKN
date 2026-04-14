#pragma once

#include "gnc/core/component_factory.hpp"
#include "gnc/core/plugin_registry.hpp"
#include "gnc/plugins/state_3dof/components/point_mass_cartesian.hpp"
#include "gnc/plugins/state_3dof/components/point_mass_spherical.hpp"

namespace gnc::plugins::state_3dof {

class State3DoFPlugin final : public gnc::core::Plugin {
public:
    const char* name() const override { return "state_3dof"; }
    gnc::core::PluginLayer layer() const override {
        return gnc::core::PluginLayer::Subsystem;
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
        factory.registerType<PointMassSpherical,
                             gnc::interfaces::IContinuousSystem,
                             IStateSolver3DOF,
                             gnc::plugins::soviet_coord::IVelocityDirectionProvider,
                             gnc::interfaces::IObservable>(
            "state_3dof.point_mass_spherical",
            gnc::core::ComponentCategory::Builtin,
            __FILE__);
    }
};

GNC_REGISTER_PLUGIN(State3DoFPlugin)

} // namespace gnc::plugins::state_3dof
