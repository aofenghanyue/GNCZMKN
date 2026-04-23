#pragma once

#include <string_view>

namespace gnc::services::coordinate_tree {

class CoordinateTreeBuilder;

namespace internal {
class CoordinateTreeBuildContext;
}

class ICoordinateTreeSpec {
public:
    virtual ~ICoordinateTreeSpec() = default;

    virtual std::string_view id() const = 0;
    virtual void build(CoordinateTreeBuilder& builder,
                       const internal::CoordinateTreeBuildContext& context) const = 0;
};

} // namespace gnc::services::coordinate_tree
