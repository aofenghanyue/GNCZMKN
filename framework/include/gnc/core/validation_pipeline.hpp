#pragma once

#include "gnc/core/component_registry.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/infrastructure/dependency_validator.hpp"

#include <string>
#include <unordered_set>
#include <vector>

namespace gnc::core {

class ValidationPipeline {
public:
    struct Result {
        std::vector<std::string> errors;
        std::vector<std::string> warnings;
        std::unordered_set<std::string> failed_components;

        bool success() const {
            return errors.empty();
        }
    };

    static Result run(ComponentRegistry& registry) {
        const auto dependency_validation = DependencyValidator::validate(registry);

        Result result;
        result.errors = dependency_validation.errors;
        result.warnings = dependency_validation.warnings;
        result.failed_components = dependency_validation.failed_components;

        preflightDependencyBindings(registry, result);
        return result;
    }

private:
    static std::string extractScope(const std::string& full_name) {
        const auto pos = full_name.find('.');
        if (pos == std::string::npos) {
            return "";
        }
        return full_name.substr(0, pos + 1);
    }

    static std::string joinDiagnosticLines(const std::vector<std::string>& values) {
        std::string result;
        for (size_t i = 0; i < values.size(); ++i) {
            result += (i == 0 ? "  - " : "\n  - ");
            result += values[i];
        }
        return result.empty() ? "  - (none)" : result;
    }

    static void preflightDependencyBindings(ComponentRegistry& registry, Result& result) {
        for (auto* component : registry.getAllComponents()) {
            if (result.failed_components.count(component->getName()) > 0) {
                continue;
            }

            ScopedRegistry::BindingDiagnostics diagnostics;
            try {
                ScopedRegistry scoped(extractScope(component->getName()),
                                      registry,
                                      component->getName(),
                                      &diagnostics);
                component->injectDependencies(scoped);
            } catch (const std::exception& e) {
                result.errors.push_back("Component '" + component->getName() +
                                        "' failed dependency preflight: " + e.what());
                continue;
            }

            if (!diagnostics.errors.empty()) {
                result.errors.push_back("Component '" + component->getName() +
                                        "' failed dependency preflight with " +
                                        std::to_string(diagnostics.errors.size()) +
                                        " required binding issue(s):\n" +
                                        joinDiagnosticLines(diagnostics.errors));
                continue;
            }

            for (const auto& warning : diagnostics.warnings) {
                result.warnings.push_back("Component '" + component->getName() +
                                          "' optional dependency warning: " + warning);
            }

            component->markDependenciesInjectedInternal_();
        }
    }
};

} // namespace gnc::core
