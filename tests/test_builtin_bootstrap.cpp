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
            "mass.continuous_constant_rate",
            "mass.constant",
            "vehicle.process.programmed_aoa",
            "vehicle.process.coordinate_probe",
            "form.cartesian_3dof.point_mass",
            "interaction.cartesian_3dof.direct_accel",
            "form.local_spherical_3dof.point_mass",
            "form.local_spherical_3dof.flight_state_view",
            "interaction.local_spherical_3dof.direct_accel",
            "interaction.local_spherical_3dof.aero_propulsive"};

        for (const auto& type_name : required_component_types) {
            test_support::require(factory.hasType(type_name),
                                  "Missing registered builtin component type: " + type_name);
        }
        test_support::require(!factory.hasType("cavh.constant_mass"),
                              "Hardcoded CAVH mass primitive should no longer be a builtin type.");
        test_support::require(!factory.hasType("cavh.aero_table"),
                              "Hardcoded CAVH aero primitive should no longer be a builtin type.");

        std::cout << "builtin bootstrap inventory checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
