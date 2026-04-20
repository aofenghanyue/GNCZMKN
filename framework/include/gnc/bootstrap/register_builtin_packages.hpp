#pragma once

#include "gnc/core/component_factory.hpp"
#include "gnc/interfaces/i_continuous_system.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/plugins/aero/components/simple_polynomial_aero.hpp"
#include "gnc/plugins/cavh/components/aero_table.hpp"
#include "gnc/plugins/cavh/components/constant_mass.hpp"
#include "gnc/plugins/environment/components/spherical_earth.hpp"
#include "gnc/plugins/environment/components/spherical_gravity.hpp"
#include "gnc/plugins/environment/components/standard_atmosphere.hpp"
#include "gnc/plugins/environment/components/wgs84_earth.hpp"
#include "gnc/plugins/flight_state_3dof/components/soviet_observer.hpp"
#include "gnc/plugins/mass/components/continuous_constant_rate_mass.hpp"
#include "gnc/plugins/state_3dof/components/point_mass_cartesian.hpp"
#include "gnc/plugins/state_3dof/components/point_mass_spherical_soviet.hpp"
#include "gnc/plugins/state_3dof_bridge/components/force_to_local_acceleration_soviet.hpp"

namespace gnc::bootstrap {

inline void registerEnvironmentPackages(gnc::core::ComponentFactory& factory) {
    using namespace gnc::core;
    factory.registerType<gnc::plugins::environment::SphericalEarth,
                         gnc::plugins::environment::IEarth>(
        "environment.spherical_earth",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Environment,
        ExecutionStage::Environment);
    factory.registerType<gnc::plugins::environment::Wgs84Earth,
                         gnc::plugins::environment::IEarth>(
        "environment.wgs84_earth",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Environment,
        ExecutionStage::Environment);
    factory.registerType<gnc::plugins::environment::StandardAtmosphere,
                         gnc::plugins::environment::IAtmosphere>(
        "environment.standard_atmosphere",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Environment,
        ExecutionStage::Environment);
    factory.registerType<gnc::plugins::environment::SphericalGravity,
                         gnc::plugins::environment::IGravity>(
        "environment.spherical_gravity",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Environment,
        ExecutionStage::Environment);
}

inline void registerAeroPackages(gnc::core::ComponentFactory& factory) {
    using namespace gnc::core;
    factory.registerType<gnc::plugins::aero::SimplePolynomialAero,
                         gnc::plugins::aero::IAeroModel,
                         gnc::interfaces::IObservable>(
        "aero.simple_polynomial",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleCommon,
        ExecutionStage::None);
}

inline void registerMassPackages(gnc::core::ComponentFactory& factory) {
    using namespace gnc::core;
    factory.registerType<gnc::plugins::mass::ContinuousConstantRateMass,
                         gnc::plugins::mass::IContinuousMass,
                         gnc::interfaces::IContinuousSystem,
                         gnc::interfaces::IObservable>(
        "mass.continuous_constant_rate",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleCommon,
        ExecutionStage::None);
}

inline void registerCavhPackages(gnc::core::ComponentFactory& factory) {
    using namespace gnc::core;
    factory.registerType<gnc::plugins::cavh::ConstantMass,
                         gnc::plugins::mass::IConstantMass,
                         gnc::interfaces::IObservable>(
        "cavh.constant_mass",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleCommon,
        ExecutionStage::None);
    factory.registerType<gnc::plugins::cavh::AeroTable,
                         gnc::plugins::aero::IAeroModel,
                         gnc::interfaces::IObservable>(
        "cavh.aero_table",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleCommon,
        ExecutionStage::None);
}

inline void registerState3DoFPackages(gnc::core::ComponentFactory& factory) {
    using namespace gnc::core;
    factory.registerType<gnc::plugins::state_3dof::PointMassCartesian,
                         gnc::interfaces::IContinuousSystem,
                         gnc::plugins::state_3dof::IStateSolver3DOF,
                         gnc::interfaces::IObservable>(
        "state_3dof.point_mass_cartesian",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Form,
        ExecutionStage::Form,
        "cartesian_3dof");
    factory.registerType<gnc::plugins::state_3dof::PointMassSphericalSoviet,
                         gnc::interfaces::IContinuousSystem,
                         gnc::plugins::state_3dof::IStateSolver3DOF,
                         gnc::plugins::state_3dof::ISovietSphericalState3DOF,
                         gnc::plugins::state_3dof::IVelocityDirectionProvider,
                         gnc::interfaces::IObservable>(
        "state_3dof.point_mass_spherical_soviet",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Form,
        ExecutionStage::Form,
        "local_spherical_3dof");
}

inline void registerState3DoFBridgePackages(gnc::core::ComponentFactory& factory) {
    using namespace gnc::core;
    factory.registerType<gnc::plugins::state_3dof_bridge::ForceToLocalAccelerationSoviet,
                         gnc::plugins::state_3dof::IAccelerationProvider3DOF>(
        "state_3dof_bridge.force_to_local_acceleration_soviet",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Interaction,
        ExecutionStage::Interaction,
        "local_spherical_3dof");
}

inline void registerFlightState3DoFPackages(gnc::core::ComponentFactory& factory) {
    using namespace gnc::core;
    factory.registerType<gnc::plugins::flight_state_3dof::SovietObserver,
                         gnc::plugins::flight_state_3dof::IFlightState3DOFSovietObserver,
                         gnc::interfaces::IObservable>(
        "flight_state_3dof.soviet_observer",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Form,
        ExecutionStage::Form,
        "local_spherical_3dof");
}

inline void registerBuiltinPackages(gnc::core::ComponentFactory& factory) {
    registerEnvironmentPackages(factory);
    registerAeroPackages(factory);
    registerMassPackages(factory);
    registerCavhPackages(factory);
    registerState3DoFPackages(factory);
    registerState3DoFBridgePackages(factory);
    registerFlightState3DoFPackages(factory);
}

} // namespace gnc::bootstrap
