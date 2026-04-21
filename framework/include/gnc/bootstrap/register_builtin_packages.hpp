#pragma once

#include "gnc/core/component_factory.hpp"
#include "gnc/environment/components/spherical_earth.hpp"
#include "gnc/environment/components/spherical_gravity.hpp"
#include "gnc/environment/components/standard_atmosphere.hpp"
#include "gnc/environment/components/wgs84_earth.hpp"
#include "gnc/environment/interfaces/i_atmosphere.hpp"
#include "gnc/environment/interfaces/i_earth.hpp"
#include "gnc/environment/interfaces/i_gravity.hpp"
#include "gnc/forms/local_spherical_3dof/components/flight_state_view.hpp"
#include "gnc/forms/local_spherical_3dof/components/point_mass.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_input_provider.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_truth_view.hpp"
#include "gnc/interfaces/i_continuous_system.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/interactions/local_spherical_3dof/components/aero_propulsive.hpp"
#include "gnc/interactions/local_spherical_3dof/components/direct_accel.hpp"
#include "gnc/plugins/state_3dof/components/point_mass_cartesian.hpp"
#include "gnc/vehicle/common/components/cavh_aero_table.hpp"
#include "gnc/vehicle/common/components/cavh_constant_mass.hpp"
#include "gnc/vehicle/common/components/continuous_constant_rate_mass.hpp"
#include "gnc/vehicle/common/components/simple_polynomial_aero.hpp"
#include "gnc/vehicle/common/interfaces/i_aero_model.hpp"
#include "gnc/vehicle/common/interfaces/i_constant_mass.hpp"
#include "gnc/vehicle/common/interfaces/i_continuous_mass.hpp"
#include "gnc/vehicle/process/components/coordinate_probe.hpp"
#include "gnc/vehicle/process/components/programmed_aoa_guidance.hpp"

namespace gnc::bootstrap {

inline void registerEnvironmentPackages(gnc::core::ComponentFactory& factory) {
    using namespace gnc::core;
    factory.registerType<gnc::environment::SphericalEarth,
                         gnc::environment::IEarth>(
        "environment.spherical_earth",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Environment,
        ExecutionStage::Environment);
    factory.registerType<gnc::environment::Wgs84Earth,
                         gnc::environment::IEarth>(
        "environment.wgs84_earth",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Environment,
        ExecutionStage::Environment);
    factory.registerType<gnc::environment::StandardAtmosphere,
                         gnc::environment::IAtmosphere>(
        "environment.standard_atmosphere",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Environment,
        ExecutionStage::Environment);
    factory.registerType<gnc::environment::SphericalGravity,
                         gnc::environment::IGravity>(
        "environment.spherical_gravity",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Environment,
        ExecutionStage::Environment);
}

inline void registerVehicleCommonPackages(gnc::core::ComponentFactory& factory) {
    using namespace gnc::core;
    factory.registerType<gnc::vehicle::common::SimplePolynomialAero,
                         gnc::vehicle::common::IAeroModel,
                         gnc::interfaces::IObservable>(
        "aero.simple_polynomial",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleCommon,
        ExecutionStage::VehicleOutput);
    factory.registerType<gnc::vehicle::common::ContinuousConstantRateMass,
                         gnc::vehicle::common::IContinuousMass,
                         gnc::interfaces::IContinuousSystem,
                         gnc::interfaces::IObservable>(
        "mass.continuous_constant_rate",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleCommon,
        ExecutionStage::VehicleOutput);
    factory.registerType<gnc::vehicle::common::CavhConstantMass,
                         gnc::vehicle::common::IConstantMass,
                         gnc::interfaces::IObservable>(
        "cavh.constant_mass",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleCommon,
        ExecutionStage::VehicleOutput);
    factory.registerType<gnc::vehicle::common::CavhAeroTable,
                         gnc::vehicle::common::IAeroModel,
                         gnc::interfaces::IObservable>(
        "cavh.aero_table",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleCommon,
        ExecutionStage::VehicleOutput);
}

inline void registerVehicleProcessPackages(gnc::core::ComponentFactory& factory) {
    using namespace gnc::core;
    factory.registerType<gnc::vehicle::process::ProgrammedAoAGuidance,
                         gnc::plugins::state_3dof::IFlightCommandProvider3DOF,
                         gnc::interfaces::IObservable>(
        "vehicle.process.programmed_aoa",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleProcess,
        ExecutionStage::VehicleProcess,
        "local_spherical_3dof");
    factory.registerType<gnc::vehicle::process::CoordinateProbe,
                         gnc::interfaces::IObservable>(
        "vehicle.process.coordinate_probe",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleProcess,
        ExecutionStage::VehicleProcess,
        "local_spherical_3dof");
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

    factory.registerType<gnc::forms::local_spherical_3dof::PointMass,
                         gnc::interfaces::IContinuousSystem,
                         gnc::plugins::state_3dof::IStateSolver3DOF,
                         gnc::plugins::state_3dof::ISovietSphericalState3DOF,
                         gnc::plugins::state_3dof::IVelocityDirectionProvider,
                         gnc::forms::local_spherical_3dof::ITruthView,
                         gnc::interfaces::IObservable>(
        "form.local_spherical_3dof.point_mass",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Form,
        ExecutionStage::Form,
        "local_spherical_3dof");

    factory.registerType<gnc::forms::local_spherical_3dof::PointMass,
                         gnc::interfaces::IContinuousSystem,
                         gnc::plugins::state_3dof::IStateSolver3DOF,
                         gnc::plugins::state_3dof::ISovietSphericalState3DOF,
                         gnc::plugins::state_3dof::IVelocityDirectionProvider,
                         gnc::forms::local_spherical_3dof::ITruthView,
                         gnc::interfaces::IObservable>(
        "state_3dof.point_mass_spherical_soviet",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Form,
        ExecutionStage::Form,
        "local_spherical_3dof");
}

inline void registerInteractionPackages(gnc::core::ComponentFactory& factory) {
    using namespace gnc::core;
    factory.registerType<gnc::interactions::local_spherical_3dof::DirectAccel,
                         gnc::forms::local_spherical_3dof::IInputProvider>(
        "interaction.local_spherical_3dof.direct_accel",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Interaction,
        ExecutionStage::Interaction,
        "local_spherical_3dof");

    factory.registerType<gnc::interactions::local_spherical_3dof::AeroPropulsive,
                         gnc::forms::local_spherical_3dof::IInputProvider,
                         gnc::plugins::state_3dof::IAccelerationProvider3DOF>(
        "interaction.local_spherical_3dof.aero_propulsive",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Interaction,
        ExecutionStage::Interaction,
        "local_spherical_3dof");

    factory.registerType<gnc::interactions::local_spherical_3dof::AeroPropulsive,
                         gnc::forms::local_spherical_3dof::IInputProvider,
                         gnc::plugins::state_3dof::IAccelerationProvider3DOF>(
        "state_3dof_bridge.force_to_local_acceleration_soviet",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Interaction,
        ExecutionStage::Interaction,
        "local_spherical_3dof");
}

inline void registerFormViewPackages(gnc::core::ComponentFactory& factory) {
    using namespace gnc::core;
    factory.registerType<gnc::forms::local_spherical_3dof::FlightStateView,
                         gnc::plugins::flight_state_3dof::IFlightState3DOFSovietObserver,
                         gnc::interfaces::IObservable>(
        "form.local_spherical_3dof.flight_state_view",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Form,
        ExecutionStage::Form,
        "local_spherical_3dof");

    factory.registerType<gnc::forms::local_spherical_3dof::FlightStateView,
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
    registerVehicleCommonPackages(factory);
    registerVehicleProcessPackages(factory);
    registerState3DoFPackages(factory);
    registerInteractionPackages(factory);
    registerFormViewPackages(factory);
}

} // namespace gnc::bootstrap
