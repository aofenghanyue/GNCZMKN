#pragma once

#include "gnc/services/coordinate_tree/internal/coordinate_tree_spec_registry.hpp"
#include "gnc/services/coordinate_tree/specs/empty_spec.hpp"
#include "gnc/services/coordinate_tree/specs/local_spherical_3dof_launch_track_spec.hpp"

#include <memory>

namespace gnc::services::coordinate_tree::specs {

inline void registerBuiltinCoordinateTreeSpecs(
    gnc::services::coordinate_tree::internal::CoordinateTreeSpecRegistry& registry) {
    registry.registerSpec(std::make_unique<EmptySpec>());
    registry.registerSpec(std::make_unique<LocalSpherical3DoFLaunchTrackSpec>());
}

} // namespace gnc::services::coordinate_tree::specs
