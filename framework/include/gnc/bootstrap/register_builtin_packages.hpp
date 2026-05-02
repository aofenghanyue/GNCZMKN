#pragma once

#include "gnc/core/component_factory.hpp"
#include "gnc/environment/components/spherical_earth.hpp"
#include "gnc/environment/components/spherical_gravity.hpp"
#include "gnc/environment/components/standard_atmosphere.hpp"
#include "gnc/environment/components/wgs84_earth.hpp"
#include "gnc/environment/interfaces/i_atmosphere.hpp"
#include "gnc/environment/interfaces/i_earth.hpp"
#include "gnc/environment/interfaces/i_gravity.hpp"
#include "gnc/forms/cartesian_3dof/components/point_mass.hpp"
#include "gnc/forms/cartesian_3dof/interfaces/i_input_provider.hpp"
#include "gnc/forms/cartesian_3dof/interfaces/i_truth_view.hpp"
#include "gnc/forms/local_spherical_3dof/components/flight_state_view.hpp"
#include "gnc/forms/local_spherical_3dof/components/point_mass.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_flight_state_view.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_input_provider.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_truth_view.hpp"
#include "gnc/interfaces/i_continuous_system.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/interactions/cartesian_3dof/components/direct_accel.hpp"
#include "gnc/interactions/cartesian_3dof/components/force_accel.hpp"
#include "gnc/interactions/local_spherical_3dof/components/aero_propulsive.hpp"
#include "gnc/interactions/local_spherical_3dof/components/direct_accel.hpp"
#include "gnc/interfaces/i_termination_evaluator.hpp"
#include "gnc/perturbation/components/static_perturbation.hpp"
#include "gnc/perturbation/interfaces/i_perturbation_provider.hpp"
#include "gnc/termination/components/component_field_threshold.hpp"
#include "gnc/vehicle/process/components/programmed_aoa_guidance.hpp"
#include "gnc/vehicle/process/interfaces/i_aero_guidance_provider.hpp"
#include "gnc/vehicle/output/components/constant_force.hpp"
#include "gnc/vehicle/output/components/constant_mass.hpp"
#include "gnc/vehicle/output/components/continuous_constant_rate_mass.hpp"
#include "gnc/vehicle/output/components/simple_polynomial_aero.hpp"
#include "gnc/vehicle/output/components/table2d_aero.hpp"
#include "gnc/vehicle/output/interfaces/i_aero_model.hpp"
#include "gnc/vehicle/output/interfaces/i_constant_mass.hpp"
#include "gnc/vehicle/output/interfaces/i_continuous_mass.hpp"
#include "gnc/vehicle/output/interfaces/i_force_provider.hpp"

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
    (void)factory;
}

inline void registerVehicleInputPackages(gnc::core::ComponentFactory& factory) {
    (void)factory;
}

inline void registerVehicleProcessPackages(gnc::core::ComponentFactory& factory) {
    using namespace gnc::core;
    factory.registerType<gnc::vehicle::process::ProgrammedAoAGuidance,
                         gnc::vehicle::process::IAeroGuidanceProvider,
                         gnc::interfaces::IObservable>(
        "vehicle.process.programmed_aoa",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleProcess,
        ExecutionStage::VehicleProcess,
        "local_spherical_3dof");
}

inline void registerPerturbationPackages(gnc::core::ComponentFactory& factory) {
    using namespace gnc::core;
    factory.registerType<gnc::perturbation::StaticPerturbation,
                         gnc::perturbation::IPerturbationProvider,
                         gnc::perturbation::IPerturbationSnapshot>(
        "perturbation.static",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Perturbation,
        ExecutionStage::Perturbation);
}

inline void registerVehicleOutputPackages(gnc::core::ComponentFactory& factory) {
    using namespace gnc::core;
    factory.registerType<gnc::vehicle::output::SimplePolynomialAero,
                         gnc::vehicle::output::IAeroModel,
                         gnc::interfaces::IObservable>(
        "aero.simple_polynomial",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleOutput,
        ExecutionStage::VehicleOutput);
    factory.registerType<gnc::vehicle::output::Table2DAero,
                         gnc::vehicle::output::IAeroModel,
                         gnc::interfaces::IObservable>(
        "aero.table2d",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleOutput,
        ExecutionStage::VehicleOutput);
    factory.registerType<gnc::vehicle::output::ContinuousConstantRateMass,
                         gnc::vehicle::output::IContinuousMass,
                         gnc::interfaces::IContinuousSystem,
                         gnc::interfaces::IObservable>(
        "mass.continuous_constant_rate",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleOutput,
        ExecutionStage::VehicleOutput);
    factory.registerType<gnc::vehicle::output::ConstantMass,
                         gnc::vehicle::output::IConstantMass,
                         gnc::interfaces::IObservable>(
        "mass.constant",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleOutput,
        ExecutionStage::VehicleOutput);
    factory.registerType<gnc::vehicle::output::ConstantForce,
                         gnc::vehicle::output::IForceProvider,
                         gnc::interfaces::IObservable>(
        "force.constant",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleOutput,
        ExecutionStage::VehicleOutput);
}

inline void registerState3DoFPackages(gnc::core::ComponentFactory& factory) {
    using namespace gnc::core;
    factory.registerType<gnc::forms::cartesian_3dof::PointMass,
                         gnc::interfaces::IContinuousSystem,
                         gnc::forms::cartesian_3dof::ITruthView,
                         gnc::interfaces::IObservable>(
        "form.cartesian_3dof.point_mass",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Form,
        ExecutionStage::Form,
        "cartesian_3dof");

    factory.registerType<gnc::forms::local_spherical_3dof::PointMass,
                         gnc::interfaces::IContinuousSystem,
                         gnc::forms::local_spherical_3dof::ITruthView,
                         gnc::interfaces::IObservable>(
        "form.local_spherical_3dof.point_mass",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Form,
        ExecutionStage::Form,
        "local_spherical_3dof");
}

inline void registerInteractionPackages(gnc::core::ComponentFactory& factory) {
    using namespace gnc::core;
    factory.registerType<gnc::interactions::cartesian_3dof::DirectAccel,
                         gnc::forms::cartesian_3dof::IInputProvider>(
        "interaction.cartesian_3dof.direct_accel",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Interaction,
        ExecutionStage::Interaction,
        "cartesian_3dof");
    factory.registerType<gnc::interactions::cartesian_3dof::ForceAccel,
                         gnc::forms::cartesian_3dof::IInputProvider>(
        "interaction.cartesian_3dof.force_accel",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Interaction,
        ExecutionStage::Interaction,
        "cartesian_3dof");

    factory.registerType<gnc::interactions::local_spherical_3dof::DirectAccel,
                         gnc::forms::local_spherical_3dof::IInputProvider>(
        "interaction.local_spherical_3dof.direct_accel",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Interaction,
        ExecutionStage::Interaction,
        "local_spherical_3dof");

    factory.registerType<gnc::interactions::local_spherical_3dof::AeroPropulsive,
                         gnc::forms::local_spherical_3dof::IInputProvider>(
        "interaction.local_spherical_3dof.aero_propulsive",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Interaction,
        ExecutionStage::Interaction,
        "local_spherical_3dof");
}

inline void registerFormViewPackages(gnc::core::ComponentFactory& factory) {
    using namespace gnc::core;
    factory.registerType<gnc::forms::local_spherical_3dof::FlightStateView,
                         gnc::forms::local_spherical_3dof::IFlightStateView,
                         gnc::interfaces::IObservable>(
        "form.local_spherical_3dof.flight_state_view",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Form,
        ExecutionStage::Form,
        "local_spherical_3dof");
}

inline void registerTerminationPackages(gnc::core::ComponentFactory& factory) {
    using namespace gnc::core;
    factory.registerType<gnc::termination::ComponentFieldBelow,
                         gnc::interfaces::ITerminationEvaluator>(
        "termination.component_field_below",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Termination,
        ExecutionStage::Termination);
    factory.registerType<gnc::termination::ComponentFieldAbove,
                         gnc::interfaces::ITerminationEvaluator>(
        "termination.component_field_above",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Termination,
        ExecutionStage::Termination);
}

inline void registerBuiltinPackages(gnc::core::ComponentFactory& factory) {
    registerEnvironmentPackages(factory);
    registerVehicleCommonPackages(factory);
    registerVehicleInputPackages(factory);
    registerPerturbationPackages(factory);
    registerVehicleProcessPackages(factory);
    registerVehicleOutputPackages(factory);
    registerState3DoFPackages(factory);
    registerInteractionPackages(factory);
    registerFormViewPackages(factory);
    registerTerminationPackages(factory);
}

} // namespace gnc::bootstrap
