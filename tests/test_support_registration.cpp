#include "test_support.hpp"

#include "auto_registered_test_components.hpp"
#include "gnc/bootstrap/register_builtin_packages.hpp"
#include "gnc/core/component_factory.hpp"

namespace test_support {

void registerBuiltinComponentTypes() {
    auto& factory = gnc::core::ComponentFactory::instance();
    gnc::bootstrap::registerBuiltinPackages(factory);
}

void registerStableTestProjectComponentTypes() {
    auto& factory = gnc::core::ComponentFactory::instance();
    gnc::build::registerAutoRegisteredTestComponents(factory);
}

void registerAvailableComponentTypes() {
    registerBuiltinComponentTypes();
}

} // namespace test_support
