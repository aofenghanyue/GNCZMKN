#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/interfaces/i_observable.hpp"

namespace tests::project_components {

class ProcessWrongStageProbe final : public gnc::core::ComponentBase,
                                     public gnc::interfaces::IObservable {
public:
    ProcessWrongStageProbe() : ComponentBase("StableTestProcessWrongStageProbe") {}

    void update(double) override {}

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        return {};
    }
};

} // namespace tests::project_components

GNC_REGISTER_COMPONENT_TYPE(
    "test_fixture.process_wrong_stage_probe",
    tests::project_components::ProcessWrongStageProbe,
    ::gnc::core::ComponentPackageRole::VehicleProcess,
    ::gnc::core::ExecutionStage::VehicleOutput,
    "",
    ::gnc::interfaces::IObservable)
