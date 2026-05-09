#pragma once

#include "gnc/simflow/materializers/numeric_perturbation_materializer.hpp"
#include "gnc/simflow/simflow_materializer_registry.hpp"

namespace gnc::bootstrap {

inline void registerBuiltinSimFlowMaterializers(
    gnc::simflow::SimFlowMaterializerRegistry& registry) {
    if (!registry.hasType("simflow.materializer.numeric_perturbation")) {
        registry.registerType<gnc::simflow::NumericPerturbationMaterializer>(
            "simflow.materializer.numeric_perturbation");
    }
}

} // namespace gnc::bootstrap
