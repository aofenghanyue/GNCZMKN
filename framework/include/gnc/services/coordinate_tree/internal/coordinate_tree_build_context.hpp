#pragma once

#include "gnc/core/config_manager.hpp"
#include "gnc/core/scoped_registry.hpp"

#include <string>

namespace gnc::services::coordinate_tree::internal {

class CoordinateTreeBuildContext {
public:
    CoordinateTreeBuildContext(gnc::core::ComponentRegistry& registry,
                               std::string scope,
                               const gnc::core::ConfigNode& config)
        : registry_(scope, registry, "coordinate_tree_service"),
          scope_(std::move(scope)),
          config_(config) {}

    gnc::core::ScopedRegistry& registry() {
        return registry_;
    }

    const gnc::core::ScopedRegistry& registry() const {
        return registry_;
    }

    const std::string& scope() const {
        return scope_;
    }

    const gnc::core::ConfigNode& config() const {
        return config_;
    }

private:
    mutable gnc::core::ScopedRegistry registry_;
    std::string scope_;
    const gnc::core::ConfigNode& config_;
};

} // namespace gnc::services::coordinate_tree::internal
