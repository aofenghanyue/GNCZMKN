#pragma once

#include "gnc/common/string_utils.hpp"
#include "gnc/core/service_package_registry.hpp"
#include "gnc/services/coordinate_tree/components/coordinate_tree_builder.hpp"
#include "gnc/services/coordinate_tree/components/coordinate_tree_service.hpp"
#include "gnc/services/coordinate_tree/internal/coordinate_tree_build_context.hpp"
#include "gnc/services/coordinate_tree/internal/coordinate_tree_spec_registry.hpp"
#include "gnc/services/coordinate_tree/specs/register_builtin_specs.hpp"

#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace gnc::services::coordinate_tree {

namespace internal {

inline std::string joinStrings(const std::vector<std::string>& values) {
    std::string result;
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
            result += ", ";
        }
        result += values[i];
    }
    return result.empty() ? "(none)" : result;
}

inline void collectUnusedConfigKeys(const gnc::core::ConfigNode& node,
                                    const std::string& path,
                                    std::vector<std::string>& unused_keys) {
    if (!node.isObject()) {
        return;
    }

    for (const auto& key : node.getUnusedKeys()) {
        unused_keys.push_back(path.empty() ? key : path + "." + key);
    }

    for (const auto& [key, child] : node) {
        const std::string child_path = path.empty() ? key : path + "." + key;
        collectUnusedConfigKeys(child, child_path, unused_keys);
    }
}

inline void warnUnusedServiceConfigKeys(
    const gnc::core::ConfigNode& config,
    const std::string& path,
    const gnc::core::ServiceDiagnosticReporter& add_warning) {
    if (!add_warning) {
        return;
    }

    std::vector<std::string> unused_keys;
    collectUnusedConfigKeys(config, path, unused_keys);
    if (unused_keys.empty()) {
        return;
    }

    add_warning("Service configuration has unrecognized keys: " +
                joinStrings(unused_keys) + ".");
}

class CoordinateTreeFinalizationTask final
    : public gnc::core::IServiceFinalizationTask {
public:
    CoordinateTreeFinalizationTask(
        std::shared_ptr<CoordinateTreeService> service,
        gnc::core::ConfigNode config,
        std::string spec_id,
        std::string registry_scope,
        std::string scope_name,
        std::string config_path,
        const CoordinateTreeSpecRegistry& specs)
        : service_(std::move(service)),
          config_(std::move(config)),
          spec_id_(std::move(spec_id)),
          registry_scope_(std::move(registry_scope)),
          scope_name_(std::move(scope_name)),
          config_path_(std::move(config_path)),
          specs_(specs) {}

    void finalize(gnc::core::ServiceFinalizationContext& context) override {
        try {
            if (!service_) {
                throw std::runtime_error("coordinate_tree pending service handle is null.");
            }

            service_->beginBuild();
            const auto* spec = specs_.findSpec(spec_id_);
            if (!spec) {
                const auto available_specs = specs_.listSpecIds();
                std::string message =
                    "Service 'coordinate_tree' in scope '" + scope_name_ +
                    "' references unknown spec '" + spec_id_ + "'.";
                if (!available_specs.empty()) {
                    message += " Available specs: " + joinStrings(available_specs) + ".";
                }
                const std::string suggestion =
                    gnc::common::findClosestMatch(spec_id_, available_specs);
                if (!suggestion.empty()) {
                    message += " Did you mean '" + suggestion + "'?";
                }
                throw std::runtime_error(message);
            }

            CoordinateTreeBuildContext build_context(
                context.registry, registry_scope_, config_);
            CoordinateTreeBuilder builder;
            spec->build(builder, build_context);
            service_->loadBuiltTree(builder.seal());
            warnUnusedServiceConfigKeys(config_,
                                        config_path_ + ".coordinate_tree",
                                        context.add_warning);
        } catch (const std::exception& e) {
            throw std::runtime_error("Service 'coordinate_tree' in scope '" +
                                     scope_name_ +
                                     "' failed to finalize: " + e.what());
        }
    }

private:
    std::shared_ptr<CoordinateTreeService> service_;
    gnc::core::ConfigNode config_;
    std::string spec_id_;
    std::string registry_scope_;
    std::string scope_name_;
    std::string config_path_;
    const CoordinateTreeSpecRegistry& specs_;
};

class CoordinateTreeServicePackage final : public gnc::core::IServicePackage {
public:
    CoordinateTreeServicePackage() {
        specs::registerBuiltinCoordinateTreeSpecs(specs_);
    }

    std::string_view id() const override {
        return "coordinate_tree";
    }

    std::vector<gnc::core::ServiceScopeKind> supportedScopes() const override {
        return {gnc::core::ServiceScopeKind::Vehicle};
    }

    std::unique_ptr<gnc::core::IServiceFinalizationTask> create(
        const gnc::core::ServiceCreationContext& context) const override {
        auto service = std::make_shared<CoordinateTreeService>();
        context.services.registerService<ICoordService>(service);

        auto config = context.config;
        const std::string spec_id = config["spec"].asString();
        return std::make_unique<CoordinateTreeFinalizationTask>(
            service,
            std::move(config),
            spec_id,
            context.scope.registry_scope,
            context.scope.name,
            context.scope.config_path,
            specs_);
    }

private:
    CoordinateTreeSpecRegistry specs_;
};

} // namespace internal

inline void registerServicePackage(gnc::core::ServicePackageRegistry& registry) {
    registry.registerPackage(
        std::make_unique<internal::CoordinateTreeServicePackage>());
}

} // namespace gnc::services::coordinate_tree
