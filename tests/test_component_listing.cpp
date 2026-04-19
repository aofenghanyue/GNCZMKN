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
                return info.type_name == "state_3dof.point_mass_cartesian";
            });

        test_support::require(it != infos.end(),
                              "Builtin point_mass_cartesian type was not registered.");
        test_support::require(
            containsValue(it->interface_names, "IContinuousSystem"),
            "point_mass_cartesian should advertise IContinuousSystem in verbose listings.");
        test_support::require(
            containsValue(it->interface_names, "IStateSolver3DOF"),
            "point_mass_cartesian should advertise IStateSolver3DOF in verbose listings.");
        test_support::require(
            containsValue(it->interface_names, "IObservable"),
            "point_mass_cartesian should advertise IObservable in verbose listings.");

        std::cout << "component listing checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
