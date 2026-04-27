#pragma once

#include "gnc/core/component_factory.hpp"

#include <string>

namespace gnc::core {

struct AssemblyDescriptor {
    std::string name;
    std::string type_name;
    std::string placement;
    ComponentPackageRole package_role = ComponentPackageRole::Unknown;
    ExecutionStage execution_stage = ExecutionStage::None;
    std::string form_family;
    std::string scope_id;
    std::string selected_form_family;
};

} // namespace gnc::core
