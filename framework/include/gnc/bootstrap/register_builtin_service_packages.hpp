#pragma once

#include "gnc/core/service_package_registry.hpp"
#include "gnc/services/coordinate_tree/bootstrap/register_service_package.hpp"

namespace gnc::bootstrap {

inline void registerBuiltinServicePackages(
    gnc::core::ServicePackageRegistry& registry) {
    gnc::services::coordinate_tree::registerServicePackage(registry);
}

} // namespace gnc::bootstrap
