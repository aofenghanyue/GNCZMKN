#include "test_support.hpp"

#include "gnc/core/component_factory.hpp"

#include <algorithm>
#include <exception>
#include <iostream>
#include <vector>

int main() {
    try {
        using namespace gnc::core;

        test_support::registerBuiltinComponentTypes();

        auto& factory = ComponentFactory::instance();

        const std::vector<std::string> required_component_types = {
            "environment.spherical_earth",
            "environment.wgs84_earth",
            "environment.standard_atmosphere",
            "environment.spherical_gravity",
            "form.cartesian_3dof.point_mass",
            "form.cartesian_6dof.rigid_body",
            "interaction.cartesian_3dof.standard",
            "interaction.cartesian_6dof.standard",
            "form.local_spherical_3dof.point_mass",
            "form.local_spherical_6dof.rigid_body",
            "interaction.local_spherical_3dof.standard",
            "interaction.local_spherical_6dof.standard",
            "form.target_point.kinematic",
            "vehicle.common.aero_assets_3dof.zero",
            "vehicle.common.aero_assets_6dof.zero",
            "vehicle.output.mass_3dof.constant",
            "vehicle.output.propulsion_3dof.zero",
            "vehicle.output.actuator_3dof.ideal",
            "vehicle.output.aerodynamics_3dof.zero",
            "vehicle.output.mass_properties_6dof.constant",
            "vehicle.output.propulsion_6dof.zero",
            "vehicle.output.actuator_6dof.ideal",
            "vehicle.output.aerodynamics_6dof.zero",
            "vehicle.input.cartesian_imu_3dof.ideal",
            "vehicle.input.cartesian_satellite_nav_3dof.ideal",
            "vehicle.input.cartesian_air_data_3dof.ideal",
            "vehicle.input.cartesian_seeker_3dof.ideal",
            "vehicle.process.cartesian_phase_sequencer_3dof.ideal",
            "vehicle.process.cartesian_trajectory_planner_3dof.ideal",
            "vehicle.process.cartesian_navigation_3dof.ideal",
            "vehicle.process.cartesian_target_tracking_3dof.ideal",
            "vehicle.process.cartesian_guidance_3dof.ideal",
            "vehicle.process.cartesian_flight_control_3dof.ideal",
            "vehicle.process.cartesian_control_allocation_3dof.ideal",
            "termination.engagement_cartesian_3dof",
            "summary.engagement_cartesian_3dof.ideal",
            "vehicle.input.cartesian_imu_6dof.ideal",
            "vehicle.input.cartesian_satellite_nav_6dof.ideal",
            "vehicle.input.cartesian_air_data_6dof.ideal",
            "vehicle.input.cartesian_seeker_6dof.ideal",
            "vehicle.process.cartesian_phase_sequencer_6dof.ideal",
            "vehicle.process.cartesian_trajectory_planner_6dof.ideal",
            "vehicle.process.cartesian_navigation_6dof.ideal",
            "vehicle.process.cartesian_target_tracking_6dof.ideal",
            "vehicle.process.cartesian_guidance_6dof.ideal",
            "vehicle.process.cartesian_attitude_control_6dof.ideal",
            "vehicle.process.cartesian_control_allocation_6dof.ideal",
            "termination.engagement_cartesian_6dof",
            "summary.engagement_cartesian_6dof.ideal"};

        for (const auto& type_name : required_component_types) {
            test_support::require(factory.hasType(type_name),
                                  "Missing registered builtin component type: " + type_name);
        }
        test_support::require(!factory.hasType("cavh.constant_mass"),
                              "Hardcoded CAVH mass primitive should no longer be a builtin type.");
        test_support::require(!factory.hasType("cavh.aero_table"),
                              "Hardcoded CAVH aero primitive should no longer be a builtin type.");
        test_support::require(
            !factory.hasType("vehicle.process.coordinate_probe"),
            "Coordinate probe is a demo utility and should not be a builtin package.");
        const std::vector<std::string> removed_builtin_types = {
            std::string("aero.") + "simple_polynomial",
            std::string("aero.") + "table2d",
            std::string("force.") + "constant",
            std::string("mass.") + "continuous_constant_rate",
            std::string("mass.") + "constant",
            std::string("vehicle.process.") + "programmed_aoa",
            std::string("interaction.cartesian_3dof.") + "direct_accel",
            std::string("interaction.cartesian_3dof.") + "force_accel",
            std::string("form.local_spherical_3dof.") + "flight_state_view",
            std::string("interaction.local_spherical_3dof.") + "direct_accel",
            std::string("interaction.local_spherical_3dof.") + "aero_propulsive"};
        for (const auto& type_name : removed_builtin_types) {
            test_support::require(!factory.hasType(type_name),
                                  "Removed legacy builtin is still registered: " + type_name);
        }

        std::cout << "builtin bootstrap inventory checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
