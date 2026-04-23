#pragma once

#include "gnc/services/coordinate_tree/components/coordinate_tree_builder.hpp"
#include "gnc/services/coordinate_tree/interfaces/i_coordinate_tree_spec.hpp"
#include "gnc/services/coordinate_tree/internal/coordinate_tree_build_context.hpp"

#include <string_view>

namespace gnc::services::coordinate_tree::specs {

class EmptySpec final : public ICoordinateTreeSpec {
public:
    std::string_view id() const override {
        return "empty";
    }

    void build(CoordinateTreeBuilder& builder,
               const gnc::services::coordinate_tree::internal::CoordinateTreeBuildContext&)
        const override {
        builder.setRoot("I");
    }
};

} // namespace gnc::services::coordinate_tree::specs
