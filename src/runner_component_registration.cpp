#include "auto_registered_components.hpp"
#include "gnc/bootstrap/register_builtin_packages.hpp"
#include "gnc/core/component_factory.hpp"

void registerRunnerComponentTypes(gnc::core::ComponentFactory& factory) {
    gnc::bootstrap::registerBuiltinPackages(factory);
    gnc::build::registerAutoRegisteredProjectComponents(factory);
}
