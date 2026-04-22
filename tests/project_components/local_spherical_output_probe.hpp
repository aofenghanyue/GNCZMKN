#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/interfaces/i_observable.hpp"

namespace tests::project_components {

class LocalSphericalOutputProbe final : public gnc::core::ComponentBase,
                                        public gnc::interfaces::IObservable {
public:
    LocalSphericalOutputProbe() : ComponentBase("StableTestLocalSphericalOutputProbe") {}

    void update(double) override {}

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        return {};
    }
};

} // namespace tests::project_components

GNC_REGISTER_COMPONENT_TYPE(
    "test_fixture.local_spherical_output_probe",
    tests::project_components::LocalSphericalOutputProbe,
    ::gnc::core::ComponentPackageRole::VehicleOutput,
    ::gnc::core::ExecutionStage::VehicleOutput,
    "local_spherical_3dof",
    ::gnc::interfaces::IObservable)
