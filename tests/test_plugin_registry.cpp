#include "test_support.hpp"

#include "gnc/bootstrap/install_builtin_services.hpp"
#include "gnc/core/component_factory.hpp"

#include <algorithm>
#include <exception>
#include <iostream>
#include <vector>

int main() {
    try {
        using namespace gnc::core;

        test_support::registerAvailableComponentTypes();

        auto& factory = ComponentFactory::instance();

        const std::vector<std::string> required_component_types = {
            "environment.spherical_earth",
            "environment.wgs84_earth",
            "environment.standard_atmosphere",
            "environment.spherical_gravity",
            "aero.simple_polynomial",
            "mass.continuous_constant_rate",
            "cavh.constant_mass",
            "cavh.aero_table",
            "vehicle.process.programmed_aoa",
            "vehicle.process.coordinate_probe",
            "form.cartesian_3dof.point_mass",
            "interaction.cartesian_3dof.direct_accel",
            "state_3dof.point_mass_cartesian",
            "form.local_spherical_3dof.point_mass",
            "form.local_spherical_3dof.flight_state_view",
            "interaction.local_spherical_3dof.direct_accel",
            "interaction.local_spherical_3dof.aero_propulsive",
            "state_3dof.point_mass_spherical_soviet",
            "state_3dof_bridge.force_to_local_acceleration_soviet",
            "flight_state_3dof.soviet_observer"};

        for (const auto& type_name : required_component_types) {
            test_support::require(factory.hasType(type_name),
                                  "Missing registered builtin component type: " + type_name);
        }

        const auto service_names = gnc::bootstrap::getBuiltinServiceNames();
        test_support::require(std::find(service_names.begin(),
                                        service_names.end(),
                                        "soviet_coord") != service_names.end(),
                              "Builtin services should advertise soviet_coord.");

        std::cout << "builtin bootstrap checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
