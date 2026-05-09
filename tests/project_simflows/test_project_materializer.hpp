#pragma once

#include "gnc/simflow/mission_patch.hpp"
#include "gnc/simflow/simflow_materializer.hpp"
#include "gnc/simflow/simflow_materializer_registration.hpp"

namespace gnc::tests {

class TestProjectSimFlowMaterializer final
    : public gnc::simflow::ISimFlowMaterializer {
public:
    std::vector<gnc::simflow::MaterializedCase> materialize(
        const gnc::simflow::SimFlowMaterializationContext& context) override {
        gnc::simflow::MaterializedCase out;
        out.case_id = "project_case";
        out.effective_mission = gnc::simflow::rewriteOutputDirectory(
            context.base_mission,
            (context.output_directory / "case_000001").generic_string());
        return {out};
    }
};

} // namespace gnc::tests

GNC_REGISTER_SIMFLOW_MATERIALIZER(
    gnc::tests::TestProjectSimFlowMaterializer,
    "test.simflow.project_materializer")
