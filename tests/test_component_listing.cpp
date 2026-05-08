#include "test_support.hpp"

#include "gnc/core/component_factory.hpp"

#include <algorithm>
#include <exception>
#include <iostream>
#include <vector>

namespace {

bool containsValue(const std::vector<std::string>& values, const std::string& expected) {
    return std::find(values.begin(), values.end(), expected) != values.end();
}

void requireRoleStage(
    const gnc::core::ComponentFactory::RegisteredTypeInfo& info,
    gnc::core::ComponentPackageRole role,
    gnc::core::ExecutionStage stage,
    const std::string& message) {
    test_support::require(info.package_role == role,
                          message + " has the wrong package role.");
    test_support::require(info.execution_stage == stage,
                          message + " has the wrong execution stage.");
}

} // namespace

int main() {
    try {
        test_support::registerBuiltinComponentTypes();

        using gnc::core::ComponentFactory;

        const auto infos = ComponentFactory::instance().getRegisteredTypeInfos();
        const auto findInfo = [&](const std::string& type_name) {
            return std::find_if(
                infos.begin(),
                infos.end(),
                [&](const ComponentFactory::RegisteredTypeInfo& info) {
                    return info.type_name == type_name;
                });
        };

        const auto it = findInfo("form.local_spherical_3dof.point_mass");

        test_support::require(it != infos.end(),
                              "Canonical local_spherical_3dof form type was not registered.");
        test_support::require(
            containsValue(it->interface_names, "IContinuousSystem"),
            "form.local_spherical_3dof.point_mass should advertise IContinuousSystem.");
        test_support::require(
            containsValue(it->interface_names, "ITruthView"),
            "form.local_spherical_3dof.point_mass should advertise ITruthView.");
        test_support::require(
            containsValue(it->interface_names, "IObservable"),
            "form.local_spherical_3dof.point_mass should advertise IObservable.");
        test_support::require(
            !containsValue(it->interface_names, "IStateSolver3DOF"),
            "form.local_spherical_3dof.point_mass should no longer advertise IStateSolver3DOF.");
        test_support::require(
            it->package_role == gnc::core::ComponentPackageRole::Form,
            "form.local_spherical_3dof.point_mass should be labeled as a form package.");
        test_support::require(
            it->execution_stage == gnc::core::ExecutionStage::Form,
            "form.local_spherical_3dof.point_mass should be labeled for the form execution stage.");
        test_support::require(
            it->form_family == "local_spherical_3dof",
            "form.local_spherical_3dof.point_mass should advertise the local_spherical_3dof family.");

        const auto aero_it = findInfo("aero.table2d");
        test_support::require(aero_it != infos.end(),
                              "Generic table2d aero builtin was not registered.");
        test_support::require(
            aero_it->package_role == gnc::core::ComponentPackageRole::VehicleOutput,
            "aero.table2d should be labeled as a vehicle.output package.");
        test_support::require(
            aero_it->execution_stage == gnc::core::ExecutionStage::VehicleOutput,
            "aero.table2d should execute in the vehicle.output stage.");

        const auto mass_it = findInfo("mass.constant");
        test_support::require(mass_it != infos.end(),
                              "Generic constant mass builtin was not registered.");
        test_support::require(
            mass_it->package_role == gnc::core::ComponentPackageRole::VehicleOutput,
            "mass.constant should be labeled as a vehicle.output package.");
        test_support::require(
            mass_it->execution_stage == gnc::core::ExecutionStage::VehicleOutput,
            "mass.constant should execute in the vehicle.output stage.");

        const auto continuous_mass_it = findInfo("mass.continuous_constant_rate");
        test_support::require(continuous_mass_it != infos.end(),
                              "Continuous mass builtin was not registered.");
        test_support::require(
            continuous_mass_it->package_role ==
                gnc::core::ComponentPackageRole::VehicleOutput,
            "mass.continuous_constant_rate should be labeled as a vehicle.output package.");
        test_support::require(
            continuous_mass_it->execution_stage ==
                gnc::core::ExecutionStage::VehicleOutput,
            "mass.continuous_constant_rate should execute in the vehicle.output stage.");

        const auto force_it = findInfo("force.constant");
        test_support::require(force_it != infos.end(),
                              "Constant force builtin was not registered.");
        test_support::require(
            containsValue(force_it->interface_names, "IForceProvider"),
            "force.constant should advertise IForceProvider.");
        test_support::require(
            force_it->package_role == gnc::core::ComponentPackageRole::VehicleOutput,
            "force.constant should be labeled as a vehicle.output package.");
        test_support::require(
            force_it->execution_stage == gnc::core::ExecutionStage::VehicleOutput,
            "force.constant should execute in the vehicle.output stage.");
        test_support::require(
            std::string(gnc::core::toString(gnc::core::ComponentPackageRole::Perturbation)) ==
                "perturbation",
            "Perturbation role should stringify as perturbation.");
        test_support::require(
            std::string(gnc::core::toString(gnc::core::ExecutionStage::Perturbation)) ==
                "perturbation",
            "Perturbation stage should stringify as perturbation.");

        const auto perturbation_it = findInfo("perturbation.static");
        test_support::require(perturbation_it != infos.end(),
                              "Static perturbation builtin was not registered.");
        test_support::require(
            perturbation_it->package_role == gnc::core::ComponentPackageRole::Perturbation,
            "perturbation.static should be labeled as a perturbation package.");
        test_support::require(
            perturbation_it->execution_stage == gnc::core::ExecutionStage::Perturbation,
            "perturbation.static should execute in the perturbation stage.");
        test_support::require(
            containsValue(perturbation_it->interface_names, "IPerturbationProvider"),
            "perturbation.static should advertise IPerturbationProvider.");

        const auto force_accel_it = findInfo("interaction.cartesian_3dof.force_accel");
        test_support::require(force_accel_it != infos.end(),
                              "Cartesian force_accel interaction was not registered.");
        test_support::require(
            force_accel_it->package_role == gnc::core::ComponentPackageRole::Interaction,
            "force_accel should be labeled as an interaction package.");
        test_support::require(
            force_accel_it->form_family == "cartesian_3dof",
            "force_accel should advertise the cartesian_3dof form family.");

        const auto simple_aero_it = findInfo("aero.simple_polynomial");
        test_support::require(simple_aero_it != infos.end(),
                              "Simple polynomial aero builtin was not registered.");
        test_support::require(
            simple_aero_it->package_role == gnc::core::ComponentPackageRole::VehicleOutput,
            "aero.simple_polynomial should be labeled as a vehicle.output package.");
        test_support::require(
            simple_aero_it->execution_stage == gnc::core::ExecutionStage::VehicleOutput,
            "aero.simple_polynomial should execute in the vehicle.output stage.");

        test_support::require(findInfo("cavh.constant_mass") == infos.end(),
                              "Hardcoded CAVH mass primitive should not appear in the builtin list.");
        test_support::require(findInfo("cavh.aero_table") == infos.end(),
                              "Hardcoded CAVH aero primitive should not appear in the builtin list.");

        const auto rigid_6dof_it = findInfo("form.local_spherical_6dof.rigid_body");
        test_support::require(rigid_6dof_it != infos.end(),
                              "6DOF placeholder form builtin was not registered.");
        requireRoleStage(*rigid_6dof_it,
                         gnc::core::ComponentPackageRole::Form,
                         gnc::core::ExecutionStage::Form,
                         "form.local_spherical_6dof.rigid_body");
        test_support::require(rigid_6dof_it->form_family == "local_spherical_6dof",
                              "6DOF form should advertise local_spherical_6dof.");
        test_support::require(containsValue(rigid_6dof_it->interface_names,
                                            "IContinuousSystem"),
                              "6DOF form should advertise IContinuousSystem.");
        test_support::require(containsValue(rigid_6dof_it->interface_names,
                                            "ITruthView"),
                              "6DOF form should advertise ITruthView.");

        const auto target_it = findInfo("form.target_point.kinematic");
        test_support::require(target_it != infos.end(),
                              "Target point form builtin was not registered.");
        requireRoleStage(*target_it,
                         gnc::core::ComponentPackageRole::Form,
                         gnc::core::ExecutionStage::Form,
                         "form.target_point.kinematic");
        test_support::require(target_it->form_family == "target_point",
                              "Target point form should advertise target_point.");

        const auto assets_3dof_it = findInfo("vehicle.common.aero_assets_3dof.zero");
        test_support::require(assets_3dof_it != infos.end(),
                              "3DOF zero aero assets builtin was not registered.");
        requireRoleStage(*assets_3dof_it,
                         gnc::core::ComponentPackageRole::VehicleCommon,
                         gnc::core::ExecutionStage::None,
                         "vehicle.common.aero_assets_3dof.zero");
        test_support::require(containsValue(assets_3dof_it->interface_names,
                                            "IAerodynamicAssets3Dof"),
                              "3DOF aero assets should advertise IAerodynamicAssets3Dof.");

        const std::vector<std::string> input_3dof_types = {
            "vehicle.input.imu_3dof.ideal",
            "vehicle.input.satellite_nav_3dof.ideal",
            "vehicle.input.air_data_3dof.ideal",
            "vehicle.input.seeker_3dof.ideal",
        };
        for (const auto& type_name : input_3dof_types) {
            const auto input_it = findInfo(type_name);
            test_support::require(input_it != infos.end(),
                                  type_name + " was not registered.");
            requireRoleStage(*input_it,
                             gnc::core::ComponentPackageRole::VehicleInput,
                             gnc::core::ExecutionStage::VehicleInput,
                             type_name);
        }

        const std::vector<std::string> process_3dof_types = {
            "vehicle.process.phase_sequencer_3dof.ideal",
            "vehicle.process.trajectory_planner_3dof.ideal",
            "vehicle.process.navigation_3dof.ideal",
            "vehicle.process.target_tracking_3dof.ideal",
            "vehicle.process.guidance_3dof.ideal",
            "vehicle.process.flight_control_3dof.ideal",
            "vehicle.process.control_allocation_3dof.ideal",
        };
        for (const auto& type_name : process_3dof_types) {
            const auto process_it = findInfo(type_name);
            test_support::require(process_it != infos.end(),
                                  type_name + " was not registered.");
            requireRoleStage(*process_it,
                             gnc::core::ComponentPackageRole::VehicleProcess,
                             gnc::core::ExecutionStage::VehicleProcess,
                             type_name);
            test_support::require(process_it->form_family == "local_spherical_3dof",
                                  type_name + " should advertise local_spherical_3dof.");
        }

        const std::vector<std::string> output_3dof_types = {
            "vehicle.output.mass_3dof.constant",
            "vehicle.output.propulsion_3dof.zero",
            "vehicle.output.actuator_3dof.ideal",
            "vehicle.output.aerodynamics_3dof.zero",
        };
        for (const auto& type_name : output_3dof_types) {
            const auto output_it = findInfo(type_name);
            test_support::require(output_it != infos.end(),
                                  type_name + " was not registered.");
            requireRoleStage(*output_it,
                             gnc::core::ComponentPackageRole::VehicleOutput,
                             gnc::core::ExecutionStage::VehicleOutput,
                             type_name);
        }

        const auto interaction_3dof_it =
            findInfo("interaction.local_spherical_3dof.standard");
        test_support::require(interaction_3dof_it != infos.end(),
                              "3DOF standard interaction was not registered.");
        requireRoleStage(*interaction_3dof_it,
                         gnc::core::ComponentPackageRole::Interaction,
                         gnc::core::ExecutionStage::Interaction,
                         "interaction.local_spherical_3dof.standard");
        test_support::require(interaction_3dof_it->form_family ==
                                  "local_spherical_3dof",
                              "3DOF interaction should advertise local_spherical_3dof.");

        const auto engagement_3dof_termination_it =
            findInfo("termination.engagement_3dof");
        test_support::require(engagement_3dof_termination_it != infos.end(),
                              "3DOF engagement termination was not registered.");
        requireRoleStage(*engagement_3dof_termination_it,
                         gnc::core::ComponentPackageRole::Termination,
                         gnc::core::ExecutionStage::Termination,
                         "termination.engagement_3dof");

        const auto engagement_3dof_summary_it =
            findInfo("summary.engagement_3dof.ideal");
        test_support::require(engagement_3dof_summary_it != infos.end(),
                              "3DOF engagement summary was not registered.");
        requireRoleStage(*engagement_3dof_summary_it,
                         gnc::core::ComponentPackageRole::Summary,
                         gnc::core::ExecutionStage::Summary,
                         "summary.engagement_3dof.ideal");

        const auto assets_it = findInfo("vehicle.common.aero_assets_6dof.zero");
        test_support::require(assets_it != infos.end(),
                              "6DOF zero aero assets builtin was not registered.");
        requireRoleStage(*assets_it,
                         gnc::core::ComponentPackageRole::VehicleCommon,
                         gnc::core::ExecutionStage::None,
                         "vehicle.common.aero_assets_6dof.zero");
        test_support::require(containsValue(assets_it->interface_names,
                                            "IAerodynamicAssets6Dof"),
                              "6DOF aero assets should advertise IAerodynamicAssets6Dof.");

        const std::vector<std::string> input_types = {
            "vehicle.input.imu_6dof.ideal",
            "vehicle.input.satellite_nav_6dof.ideal",
            "vehicle.input.air_data_6dof.ideal",
            "vehicle.input.seeker_6dof.ideal",
        };
        for (const auto& type_name : input_types) {
            const auto input_it = findInfo(type_name);
            test_support::require(input_it != infos.end(),
                                  type_name + " was not registered.");
            requireRoleStage(*input_it,
                             gnc::core::ComponentPackageRole::VehicleInput,
                             gnc::core::ExecutionStage::VehicleInput,
                             type_name);
        }

        const std::vector<std::string> process_types = {
            "vehicle.process.phase_sequencer_6dof.ideal",
            "vehicle.process.trajectory_planner_6dof.ideal",
            "vehicle.process.navigation_6dof.ideal",
            "vehicle.process.target_tracking_6dof.ideal",
            "vehicle.process.guidance_6dof.ideal",
            "vehicle.process.attitude_control_6dof.ideal",
            "vehicle.process.control_allocation_6dof.ideal",
        };
        for (const auto& type_name : process_types) {
            const auto process_it = findInfo(type_name);
            test_support::require(process_it != infos.end(),
                                  type_name + " was not registered.");
            requireRoleStage(*process_it,
                             gnc::core::ComponentPackageRole::VehicleProcess,
                             gnc::core::ExecutionStage::VehicleProcess,
                             type_name);
            test_support::require(process_it->form_family == "local_spherical_6dof",
                                  type_name + " should advertise local_spherical_6dof.");
        }

        const std::vector<std::string> output_types = {
            "vehicle.output.mass_properties_6dof.constant",
            "vehicle.output.propulsion_6dof.zero",
            "vehicle.output.actuator_6dof.ideal",
            "vehicle.output.aerodynamics_6dof.zero",
        };
        for (const auto& type_name : output_types) {
            const auto output_it = findInfo(type_name);
            test_support::require(output_it != infos.end(),
                                  type_name + " was not registered.");
            requireRoleStage(*output_it,
                             gnc::core::ComponentPackageRole::VehicleOutput,
                             gnc::core::ExecutionStage::VehicleOutput,
                             type_name);
        }

        const auto interaction_6dof_it =
            findInfo("interaction.local_spherical_6dof.standard");
        test_support::require(interaction_6dof_it != infos.end(),
                              "6DOF standard interaction was not registered.");
        requireRoleStage(*interaction_6dof_it,
                         gnc::core::ComponentPackageRole::Interaction,
                         gnc::core::ExecutionStage::Interaction,
                         "interaction.local_spherical_6dof.standard");
        test_support::require(interaction_6dof_it->form_family ==
                                  "local_spherical_6dof",
                              "6DOF interaction should advertise local_spherical_6dof.");

        const auto engagement_termination_it =
            findInfo("termination.engagement_6dof");
        test_support::require(engagement_termination_it != infos.end(),
                              "6DOF engagement termination was not registered.");
        requireRoleStage(*engagement_termination_it,
                         gnc::core::ComponentPackageRole::Termination,
                         gnc::core::ExecutionStage::Termination,
                         "termination.engagement_6dof");

        const auto engagement_summary_it =
            findInfo("summary.engagement_6dof.ideal");
        test_support::require(engagement_summary_it != infos.end(),
                              "6DOF engagement summary was not registered.");
        requireRoleStage(*engagement_summary_it,
                         gnc::core::ComponentPackageRole::Summary,
                         gnc::core::ExecutionStage::Summary,
                         "summary.engagement_6dof.ideal");

        std::cout << "component listing checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
