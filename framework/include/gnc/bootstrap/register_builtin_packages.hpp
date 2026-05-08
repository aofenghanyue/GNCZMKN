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
#include "gnc/forms/local_spherical_6dof/components/rigid_body.hpp"
#include "gnc/forms/local_spherical_6dof/interfaces/i_input_provider.hpp"
#include "gnc/forms/local_spherical_6dof/interfaces/i_truth_view.hpp"
#include "gnc/forms/target_point/components/kinematic_point.hpp"
#include "gnc/forms/target_point/interfaces/i_truth_view.hpp"
#include "gnc/interfaces/i_continuous_system.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/interactions/cartesian_3dof/components/direct_accel.hpp"
#include "gnc/interactions/cartesian_3dof/components/force_accel.hpp"
#include "gnc/interactions/cartesian_3dof/components/standard_closure.hpp"
#include "gnc/interactions/local_spherical_3dof/components/aero_propulsive.hpp"
#include "gnc/interactions/local_spherical_3dof/components/direct_accel.hpp"
#include "gnc/interactions/local_spherical_3dof/components/standard_closure.hpp"
#include "gnc/interactions/local_spherical_6dof/components/standard_closure.hpp"
#include "gnc/interfaces/i_termination_evaluator.hpp"
#include "gnc/perturbation/components/static_perturbation.hpp"
#include "gnc/perturbation/interfaces/i_perturbation_provider.hpp"
#include "gnc/termination/components/component_field_threshold.hpp"
#include "gnc/termination/components/engagement_termination_cartesian_3dof.hpp"
#include "gnc/termination/components/engagement_termination_3dof.hpp"
#include "gnc/termination/components/engagement_termination.hpp"
#include "gnc/summary/components/ideal_engagement_summary_cartesian_3dof.hpp"
#include "gnc/summary/components/ideal_engagement_summary_3dof.hpp"
#include "gnc/summary/components/ideal_engagement_summary.hpp"
#include "gnc/vehicle/input/components/ideal_cartesian_air_data_3dof.hpp"
#include "gnc/vehicle/input/components/ideal_cartesian_imu_3dof.hpp"
#include "gnc/vehicle/input/components/ideal_cartesian_satellite_nav_3dof.hpp"
#include "gnc/vehicle/input/components/ideal_cartesian_seeker_3dof.hpp"
#include "gnc/vehicle/input/components/ideal_air_data_3dof.hpp"
#include "gnc/vehicle/input/components/ideal_air_data_6dof.hpp"
#include "gnc/vehicle/input/components/ideal_imu_3dof.hpp"
#include "gnc/vehicle/input/components/ideal_imu_6dof.hpp"
#include "gnc/vehicle/input/components/ideal_satellite_nav_3dof.hpp"
#include "gnc/vehicle/input/components/ideal_satellite_nav_6dof.hpp"
#include "gnc/vehicle/input/components/ideal_seeker_3dof.hpp"
#include "gnc/vehicle/input/components/ideal_seeker_6dof.hpp"
#include "gnc/vehicle/input/interfaces/i_air_data_3dof.hpp"
#include "gnc/vehicle/input/interfaces/i_air_data_6dof.hpp"
#include "gnc/vehicle/input/interfaces/i_imu_3dof.hpp"
#include "gnc/vehicle/input/interfaces/i_imu_6dof.hpp"
#include "gnc/vehicle/input/interfaces/i_satellite_nav_3dof.hpp"
#include "gnc/vehicle/input/interfaces/i_satellite_nav_6dof.hpp"
#include "gnc/vehicle/input/interfaces/i_seeker_3dof.hpp"
#include "gnc/vehicle/input/interfaces/i_seeker_6dof.hpp"
#include "gnc/vehicle/common/components/zero_aerodynamic_assets_3dof.hpp"
#include "gnc/vehicle/common/components/zero_aerodynamic_assets_6dof.hpp"
#include "gnc/vehicle/common/interfaces/i_aerodynamic_assets_3dof.hpp"
#include "gnc/vehicle/common/interfaces/i_aerodynamic_assets_6dof.hpp"
#include "gnc/vehicle/process/components/ideal_control_allocation_3dof.hpp"
#include "gnc/vehicle/process/components/ideal_flight_control_3dof.hpp"
#include "gnc/vehicle/process/components/ideal_guidance_3dof.hpp"
#include "gnc/vehicle/process/components/ideal_navigation_3dof.hpp"
#include "gnc/vehicle/process/components/ideal_phase_sequencer_3dof.hpp"
#include "gnc/vehicle/process/components/ideal_target_tracking_3dof.hpp"
#include "gnc/vehicle/process/components/ideal_trajectory_planner_3dof.hpp"
#include "gnc/vehicle/process/components/programmed_aoa_guidance.hpp"
#include "gnc/vehicle/process/components/ideal_attitude_control_6dof.hpp"
#include "gnc/vehicle/process/components/ideal_control_allocation_6dof.hpp"
#include "gnc/vehicle/process/components/ideal_guidance_6dof.hpp"
#include "gnc/vehicle/process/components/ideal_navigation_6dof.hpp"
#include "gnc/vehicle/process/components/ideal_phase_sequencer_6dof.hpp"
#include "gnc/vehicle/process/components/ideal_target_tracking_6dof.hpp"
#include "gnc/vehicle/process/components/ideal_trajectory_planner_6dof.hpp"
#include "gnc/vehicle/process/interfaces/i_aero_guidance_provider.hpp"
#include "gnc/vehicle/process/interfaces/i_control_allocation_3dof.hpp"
#include "gnc/vehicle/process/interfaces/i_flight_control_3dof.hpp"
#include "gnc/vehicle/process/interfaces/i_guidance_3dof.hpp"
#include "gnc/vehicle/process/interfaces/i_navigation_3dof.hpp"
#include "gnc/vehicle/process/interfaces/i_phase_sequencer_3dof.hpp"
#include "gnc/vehicle/process/interfaces/i_target_tracking_3dof.hpp"
#include "gnc/vehicle/process/interfaces/i_trajectory_planner_3dof.hpp"
#include "gnc/vehicle/process/interfaces/i_attitude_control_6dof.hpp"
#include "gnc/vehicle/process/interfaces/i_control_allocation_6dof.hpp"
#include "gnc/vehicle/process/interfaces/i_guidance_6dof.hpp"
#include "gnc/vehicle/process/interfaces/i_navigation_6dof.hpp"
#include "gnc/vehicle/process/interfaces/i_phase_sequencer_6dof.hpp"
#include "gnc/vehicle/process/interfaces/i_target_tracking_6dof.hpp"
#include "gnc/vehicle/process/interfaces/i_trajectory_planner_6dof.hpp"
#include "gnc/vehicle/output/components/constant_force.hpp"
#include "gnc/vehicle/output/components/constant_mass_3dof.hpp"
#include "gnc/vehicle/output/components/constant_mass_properties_6dof.hpp"
#include "gnc/vehicle/output/components/constant_mass.hpp"
#include "gnc/vehicle/output/components/continuous_constant_rate_mass.hpp"
#include "gnc/vehicle/output/components/ideal_actuator_3dof.hpp"
#include "gnc/vehicle/output/components/ideal_actuator_6dof.hpp"
#include "gnc/vehicle/output/components/simple_polynomial_aero.hpp"
#include "gnc/vehicle/output/components/table2d_aero.hpp"
#include "gnc/vehicle/output/components/zero_aerodynamics_3dof.hpp"
#include "gnc/vehicle/output/components/zero_aerodynamics_6dof.hpp"
#include "gnc/vehicle/output/components/zero_propulsion_3dof.hpp"
#include "gnc/vehicle/output/components/zero_propulsion_6dof.hpp"
#include "gnc/vehicle/output/interfaces/i_actuator_3dof.hpp"
#include "gnc/vehicle/output/interfaces/i_actuator_6dof.hpp"
#include "gnc/vehicle/output/interfaces/i_aerodynamics_6dof.hpp"
#include "gnc/vehicle/output/interfaces/i_aero_model.hpp"
#include "gnc/vehicle/output/interfaces/i_constant_mass.hpp"
#include "gnc/vehicle/output/interfaces/i_continuous_mass.hpp"
#include "gnc/vehicle/output/interfaces/i_force_provider.hpp"
#include "gnc/vehicle/output/interfaces/i_mass_properties_6dof.hpp"
#include "gnc/vehicle/output/interfaces/i_propulsion_6dof.hpp"

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
    factory.registerType<gnc::vehicle::common::ZeroAerodynamicAssets3Dof,
                         gnc::vehicle::common::IAerodynamicAssets3Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.common.aero_assets_3dof.zero",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleCommon,
        ExecutionStage::None);
    factory.registerType<gnc::vehicle::common::ZeroAerodynamicAssets6Dof,
                         gnc::vehicle::common::IAerodynamicAssets6Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.common.aero_assets_6dof.zero",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleCommon,
        ExecutionStage::None);
}

inline void registerVehicleInputPackages(gnc::core::ComponentFactory& factory) {
    using namespace gnc::core;
    factory.registerType<gnc::vehicle::input::IdealImu3Dof,
                         gnc::vehicle::input::IImu3Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.input.imu_3dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleInput,
        ExecutionStage::VehicleInput);
    factory.registerType<gnc::vehicle::input::IdealSatelliteNav3Dof,
                         gnc::vehicle::input::ISatelliteNav3Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.input.satellite_nav_3dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleInput,
        ExecutionStage::VehicleInput);
    factory.registerType<gnc::vehicle::input::IdealAirData3Dof,
                         gnc::vehicle::input::IAirData3Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.input.air_data_3dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleInput,
        ExecutionStage::VehicleInput);
    factory.registerType<gnc::vehicle::input::IdealSeeker3Dof,
                         gnc::vehicle::input::ISeeker3Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.input.seeker_3dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleInput,
        ExecutionStage::VehicleInput);
    factory.registerType<gnc::vehicle::input::IdealCartesianImu3Dof,
                         gnc::vehicle::input::IImu3Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.input.cartesian_imu_3dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleInput,
        ExecutionStage::VehicleInput,
        "cartesian_3dof");
    factory.registerType<gnc::vehicle::input::IdealCartesianSatelliteNav3Dof,
                         gnc::vehicle::input::ISatelliteNav3Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.input.cartesian_satellite_nav_3dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleInput,
        ExecutionStage::VehicleInput,
        "cartesian_3dof");
    factory.registerType<gnc::vehicle::input::IdealCartesianAirData3Dof,
                         gnc::vehicle::input::IAirData3Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.input.cartesian_air_data_3dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleInput,
        ExecutionStage::VehicleInput,
        "cartesian_3dof");
    factory.registerType<gnc::vehicle::input::IdealCartesianSeeker3Dof,
                         gnc::vehicle::input::ISeeker3Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.input.cartesian_seeker_3dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleInput,
        ExecutionStage::VehicleInput,
        "cartesian_3dof");
    factory.registerType<gnc::vehicle::input::IdealImu6Dof,
                         gnc::vehicle::input::IImu6Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.input.imu_6dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleInput,
        ExecutionStage::VehicleInput);
    factory.registerType<gnc::vehicle::input::IdealSatelliteNav6Dof,
                         gnc::vehicle::input::ISatelliteNav6Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.input.satellite_nav_6dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleInput,
        ExecutionStage::VehicleInput);
    factory.registerType<gnc::vehicle::input::IdealAirData6Dof,
                         gnc::vehicle::input::IAirData6Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.input.air_data_6dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleInput,
        ExecutionStage::VehicleInput);
    factory.registerType<gnc::vehicle::input::IdealSeeker6Dof,
                         gnc::vehicle::input::ISeeker6Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.input.seeker_6dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleInput,
        ExecutionStage::VehicleInput);
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
    factory.registerType<gnc::vehicle::process::IdealPhaseSequencer3Dof,
                         gnc::vehicle::process::IPhaseSequencer3Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.process.phase_sequencer_3dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleProcess,
        ExecutionStage::VehicleProcess,
        "local_spherical_3dof");
    factory.registerType<gnc::vehicle::process::IdealTrajectoryPlanner3Dof,
                         gnc::vehicle::process::ITrajectoryPlanner3Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.process.trajectory_planner_3dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleProcess,
        ExecutionStage::VehicleProcess,
        "local_spherical_3dof");
    factory.registerType<gnc::vehicle::process::IdealNavigation3Dof,
                         gnc::vehicle::process::INavigation3Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.process.navigation_3dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleProcess,
        ExecutionStage::VehicleProcess,
        "local_spherical_3dof");
    factory.registerType<gnc::vehicle::process::IdealTargetTracking3Dof,
                         gnc::vehicle::process::ITargetTracking3Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.process.target_tracking_3dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleProcess,
        ExecutionStage::VehicleProcess,
        "local_spherical_3dof");
    factory.registerType<gnc::vehicle::process::IdealGuidance3Dof,
                         gnc::vehicle::process::IGuidance3Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.process.guidance_3dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleProcess,
        ExecutionStage::VehicleProcess,
        "local_spherical_3dof");
    factory.registerType<gnc::vehicle::process::IdealFlightControl3Dof,
                         gnc::vehicle::process::IFlightControl3Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.process.flight_control_3dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleProcess,
        ExecutionStage::VehicleProcess,
        "local_spherical_3dof");
    factory.registerType<gnc::vehicle::process::IdealControlAllocation3Dof,
                         gnc::vehicle::process::IControlAllocation3Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.process.control_allocation_3dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleProcess,
        ExecutionStage::VehicleProcess,
        "local_spherical_3dof");
    factory.registerType<gnc::vehicle::process::IdealPhaseSequencer3Dof,
                         gnc::vehicle::process::IPhaseSequencer3Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.process.cartesian_phase_sequencer_3dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleProcess,
        ExecutionStage::VehicleProcess,
        "cartesian_3dof");
    factory.registerType<gnc::vehicle::process::IdealTrajectoryPlanner3Dof,
                         gnc::vehicle::process::ITrajectoryPlanner3Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.process.cartesian_trajectory_planner_3dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleProcess,
        ExecutionStage::VehicleProcess,
        "cartesian_3dof");
    factory.registerType<gnc::vehicle::process::IdealNavigation3Dof,
                         gnc::vehicle::process::INavigation3Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.process.cartesian_navigation_3dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleProcess,
        ExecutionStage::VehicleProcess,
        "cartesian_3dof");
    factory.registerType<gnc::vehicle::process::IdealTargetTracking3Dof,
                         gnc::vehicle::process::ITargetTracking3Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.process.cartesian_target_tracking_3dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleProcess,
        ExecutionStage::VehicleProcess,
        "cartesian_3dof");
    factory.registerType<gnc::vehicle::process::IdealGuidance3Dof,
                         gnc::vehicle::process::IGuidance3Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.process.cartesian_guidance_3dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleProcess,
        ExecutionStage::VehicleProcess,
        "cartesian_3dof");
    factory.registerType<gnc::vehicle::process::IdealFlightControl3Dof,
                         gnc::vehicle::process::IFlightControl3Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.process.cartesian_flight_control_3dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleProcess,
        ExecutionStage::VehicleProcess,
        "cartesian_3dof");
    factory.registerType<gnc::vehicle::process::IdealControlAllocation3Dof,
                         gnc::vehicle::process::IControlAllocation3Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.process.cartesian_control_allocation_3dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleProcess,
        ExecutionStage::VehicleProcess,
        "cartesian_3dof");
    factory.registerType<gnc::vehicle::process::IdealPhaseSequencer6Dof,
                         gnc::vehicle::process::IPhaseSequencer6Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.process.phase_sequencer_6dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleProcess,
        ExecutionStage::VehicleProcess,
        "local_spherical_6dof");
    factory.registerType<gnc::vehicle::process::IdealTrajectoryPlanner6Dof,
                         gnc::vehicle::process::ITrajectoryPlanner6Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.process.trajectory_planner_6dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleProcess,
        ExecutionStage::VehicleProcess,
        "local_spherical_6dof");
    factory.registerType<gnc::vehicle::process::IdealNavigation6Dof,
                         gnc::vehicle::process::INavigation6Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.process.navigation_6dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleProcess,
        ExecutionStage::VehicleProcess,
        "local_spherical_6dof");
    factory.registerType<gnc::vehicle::process::IdealTargetTracking6Dof,
                         gnc::vehicle::process::ITargetTracking6Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.process.target_tracking_6dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleProcess,
        ExecutionStage::VehicleProcess,
        "local_spherical_6dof");
    factory.registerType<gnc::vehicle::process::IdealGuidance6Dof,
                         gnc::vehicle::process::IGuidance6Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.process.guidance_6dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleProcess,
        ExecutionStage::VehicleProcess,
        "local_spherical_6dof");
    factory.registerType<gnc::vehicle::process::IdealAttitudeControl6Dof,
                         gnc::vehicle::process::IAttitudeControl6Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.process.attitude_control_6dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleProcess,
        ExecutionStage::VehicleProcess,
        "local_spherical_6dof");
    factory.registerType<gnc::vehicle::process::IdealControlAllocation6Dof,
                         gnc::vehicle::process::IControlAllocation6Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.process.control_allocation_6dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleProcess,
        ExecutionStage::VehicleProcess,
        "local_spherical_6dof");
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
    factory.registerType<gnc::vehicle::output::ZeroPropulsion3Dof,
                         gnc::vehicle::output::IForceProvider,
                         gnc::interfaces::IObservable>(
        "vehicle.output.propulsion_3dof.zero",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleOutput,
        ExecutionStage::VehicleOutput);
    factory.registerType<gnc::vehicle::output::ConstantMass3Dof,
                         gnc::vehicle::output::IConstantMass,
                         gnc::interfaces::IObservable>(
        "vehicle.output.mass_3dof.constant",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleOutput,
        ExecutionStage::VehicleOutput);
    factory.registerType<gnc::vehicle::output::IdealActuator3Dof,
                         gnc::vehicle::output::IActuator3Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.output.actuator_3dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleOutput,
        ExecutionStage::VehicleOutput);
    factory.registerType<gnc::vehicle::output::ZeroAerodynamics3Dof,
                         gnc::vehicle::output::IAeroModel,
                         gnc::interfaces::IObservable>(
        "vehicle.output.aerodynamics_3dof.zero",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleOutput,
        ExecutionStage::VehicleOutput);
    factory.registerType<gnc::vehicle::output::ConstantMassProperties6Dof,
                         gnc::vehicle::output::IMassProperties6Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.output.mass_properties_6dof.constant",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleOutput,
        ExecutionStage::VehicleOutput);
    factory.registerType<gnc::vehicle::output::ZeroPropulsion6Dof,
                         gnc::vehicle::output::IPropulsion6Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.output.propulsion_6dof.zero",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleOutput,
        ExecutionStage::VehicleOutput);
    factory.registerType<gnc::vehicle::output::IdealActuator6Dof,
                         gnc::vehicle::output::IActuator6Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.output.actuator_6dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::VehicleOutput,
        ExecutionStage::VehicleOutput);
    factory.registerType<gnc::vehicle::output::ZeroAerodynamics6Dof,
                         gnc::vehicle::output::IAerodynamics6Dof,
                         gnc::interfaces::IObservable>(
        "vehicle.output.aerodynamics_6dof.zero",
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

    factory.registerType<gnc::forms::local_spherical_6dof::RigidBody,
                         gnc::interfaces::IContinuousSystem,
                         gnc::forms::local_spherical_6dof::ITruthView,
                         gnc::interfaces::IObservable>(
        "form.local_spherical_6dof.rigid_body",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Form,
        ExecutionStage::Form,
        "local_spherical_6dof");

    factory.registerType<gnc::forms::target_point::KinematicPoint,
                         gnc::interfaces::IContinuousSystem,
                         gnc::forms::target_point::ITruthView,
                         gnc::interfaces::IObservable>(
        "form.target_point.kinematic",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Form,
        ExecutionStage::Form,
        "target_point");
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
    factory.registerType<gnc::interactions::cartesian_3dof::StandardClosure,
                         gnc::forms::cartesian_3dof::IInputProvider,
                         gnc::interfaces::IObservable>(
        "interaction.cartesian_3dof.standard",
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

    factory.registerType<gnc::interactions::local_spherical_3dof::StandardClosure,
                         gnc::forms::local_spherical_3dof::IInputProvider,
                         gnc::interfaces::IObservable>(
        "interaction.local_spherical_3dof.standard",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Interaction,
        ExecutionStage::Interaction,
        "local_spherical_3dof");

    factory.registerType<gnc::interactions::local_spherical_6dof::StandardClosure,
                         gnc::forms::local_spherical_6dof::IInputProvider,
                         gnc::interfaces::IObservable>(
        "interaction.local_spherical_6dof.standard",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Interaction,
        ExecutionStage::Interaction,
        "local_spherical_6dof");
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
    factory.registerType<gnc::termination::EngagementTermination6Dof,
                         gnc::interfaces::ITerminationEvaluator>(
        "termination.engagement_6dof",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Termination,
        ExecutionStage::Termination);
    factory.registerType<gnc::termination::EngagementTermination3Dof,
                         gnc::interfaces::ITerminationEvaluator>(
        "termination.engagement_3dof",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Termination,
        ExecutionStage::Termination);
    factory.registerType<gnc::termination::EngagementTerminationCartesian3Dof,
                         gnc::interfaces::ITerminationEvaluator>(
        "termination.engagement_cartesian_3dof",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Termination,
        ExecutionStage::Termination);
}

inline void registerSummaryPackages(gnc::core::ComponentFactory& factory) {
    using namespace gnc::core;
    factory.registerType<gnc::summary::IdealEngagementSummary3Dof,
                         gnc::interfaces::ISummaryObserver>(
        "summary.engagement_3dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Summary,
        ExecutionStage::Summary);
    factory.registerType<gnc::summary::IdealEngagementSummaryCartesian3Dof,
                         gnc::interfaces::ISummaryObserver>(
        "summary.engagement_cartesian_3dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Summary,
        ExecutionStage::Summary);
    factory.registerType<gnc::summary::IdealEngagementSummary6Dof,
                         gnc::interfaces::ISummaryObserver>(
        "summary.engagement_6dof.ideal",
        ComponentCategory::Builtin,
        __FILE__,
        ComponentPackageRole::Summary,
        ExecutionStage::Summary);
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
    registerSummaryPackages(factory);
}

} // namespace gnc::bootstrap
