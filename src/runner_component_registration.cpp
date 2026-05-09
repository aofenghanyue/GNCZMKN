#include "auto_registered_components.hpp"
#include "auto_registered_simflow_materializers.hpp"
#include "gnc/bootstrap/register_builtin_packages.hpp"
#include "gnc/bootstrap/register_builtin_simflow_materializers.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/simflow/simflow_materializer_registry.hpp"

void registerRunnerComponentTypes(gnc::core::ComponentFactory& factory) {
    gnc::bootstrap::registerBuiltinPackages(factory);
    gnc::build::registerAutoRegisteredProjectComponents(factory);
}

void registerRunnerSimFlowMaterializers(
    gnc::simflow::SimFlowMaterializerRegistry& registry) {
    gnc::bootstrap::registerBuiltinSimFlowMaterializers(registry);
    gnc::build::registerAutoRegisteredSimFlowMaterializers(registry);
}
