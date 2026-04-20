#include "test_support.hpp"

#include "gnc/core/component_factory.hpp"
#include "gnc/plugins/_builtin_plugins.hpp"

#include <algorithm>
#include <exception>
#include <iostream>
#include <vector>

namespace {

bool containsValue(const std::vector<std::string>& values, const std::string& expected) {
    return std::find(values.begin(), values.end(), expected) != values.end();
}

} // namespace

int main() {
    try {
        using gnc::core::ComponentFactory;

        const auto infos = ComponentFactory::instance().getRegisteredTypeInfos();
        const auto it = std::find_if(
            infos.begin(),
            infos.end(),
            [](const ComponentFactory::RegisteredTypeInfo& info) {
                return info.type_name == "state_3dof.point_mass_spherical_soviet";
            });

        test_support::require(it != infos.end(),
                              "Builtin Soviet spherical 3DOF type was not registered.");
        test_support::require(
            containsValue(it->interface_names, "IContinuousSystem"),
            "point_mass_spherical_soviet should advertise IContinuousSystem.");
        test_support::require(
            containsValue(it->interface_names, "IStateSolver3DOF"),
            "point_mass_spherical_soviet should advertise IStateSolver3DOF.");
        test_support::require(
            containsValue(it->interface_names, "ISovietSphericalState3DOF"),
            "point_mass_spherical_soviet should advertise ISovietSphericalState3DOF.");
        test_support::require(
            containsValue(it->interface_names, "IObservable"),
            "point_mass_spherical_soviet should advertise IObservable.");

        std::cout << "component listing checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
