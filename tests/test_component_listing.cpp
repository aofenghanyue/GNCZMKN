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

} // namespace

int main() {
    try {
        test_support::registerAvailableComponentTypes();

        using gnc::core::ComponentFactory;

        const auto infos = ComponentFactory::instance().getRegisteredTypeInfos();
        const auto it = std::find_if(
            infos.begin(),
            infos.end(),
            [](const ComponentFactory::RegisteredTypeInfo& info) {
                return info.type_name == "form.local_spherical_3dof.point_mass";
            });

        test_support::require(it != infos.end(),
                              "Canonical local_spherical_3dof form type was not registered.");
        test_support::require(
            containsValue(it->interface_names, "IContinuousSystem"),
            "form.local_spherical_3dof.point_mass should advertise IContinuousSystem.");
        test_support::require(
            containsValue(it->interface_names, "IStateSolver3DOF"),
            "form.local_spherical_3dof.point_mass should advertise IStateSolver3DOF.");
        test_support::require(
            containsValue(it->interface_names, "ISovietSphericalState3DOF"),
            "form.local_spherical_3dof.point_mass should advertise ISovietSphericalState3DOF.");
        test_support::require(
            containsValue(it->interface_names, "ITruthView"),
            "form.local_spherical_3dof.point_mass should advertise ITruthView.");
        test_support::require(
            containsValue(it->interface_names, "IObservable"),
            "form.local_spherical_3dof.point_mass should advertise IObservable.");
        test_support::require(
            it->package_role == gnc::core::ComponentPackageRole::Form,
            "form.local_spherical_3dof.point_mass should be labeled as a form package.");
        test_support::require(
            it->execution_stage == gnc::core::ExecutionStage::Form,
            "form.local_spherical_3dof.point_mass should be labeled for the form execution stage.");
        test_support::require(
            it->form_family == "local_spherical_3dof",
            "form.local_spherical_3dof.point_mass should advertise the local_spherical_3dof family.");

        std::cout << "component listing checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
