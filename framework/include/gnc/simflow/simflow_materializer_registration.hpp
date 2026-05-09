#pragma once

#include "gnc/simflow/simflow_materializer_registry.hpp"

#ifndef GNC_SIMFLOW_MATERIALIZER_REGISTRATION_FN
#define GNC_SIMFLOW_MATERIALIZER_REGISTRATION_FN gnc_register_simflow_materializer
#endif

#define GNC_REGISTER_SIMFLOW_MATERIALIZER(TYPE, TYPE_ID)                  \
    inline void GNC_SIMFLOW_MATERIALIZER_REGISTRATION_FN(                 \
        ::gnc::simflow::SimFlowMaterializerRegistry& registry) {          \
        registry.registerType<TYPE>(TYPE_ID);                             \
    }
