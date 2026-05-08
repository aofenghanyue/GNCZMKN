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
            "aero.simple_polynomial",
            "aero.table2d",
            "force.constant",
            "mass.continuous_constant_rate",
            "mass.constant",
            "vehicle.process.programmed_aoa",
            "form.cartesian_3dof.point_mass",
            "form.cartesian_6dof.rigid_body",
            "interaction.cartesian_3dof.direct_accel",
            "interaction.cartesian_3dof.force_accel",
            "interaction.cartesian_3dof.standard",
            "interaction.cartesian_6dof.standard",
            "form.local_spherical_3dof.point_mass",
            "form.local_spherical_3dof.flight_state_view",
            "interaction.local_spherical_3dof.direct_accel",
            "interaction.local_spherical_3dof.aero_propulsive",
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

        std::cout << "builtin bootstrap inventory checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
