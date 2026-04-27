#pragma once

#include "gnc/core/assembly_descriptor.hpp"
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

    static Result run(ComponentRegistry& registry,
                      const std::vector<AssemblyDescriptor>& descriptors = {},
                      const std::string& selected_form_family = "") {
        const auto dependency_validation = DependencyValidator::validate(registry);

        Result result;
        result.errors = dependency_validation.errors;
        result.warnings = dependency_validation.warnings;
        result.failed_components = dependency_validation.failed_components;

        validateAssemblyDescriptors(descriptors, selected_form_family, result);
        preflightDependencyBindings(registry, result);
        return result;
    }

private:
    static std::string roleLabel(ComponentPackageRole role) {
        return toString(role);
    }

    static std::string stageLabel(ExecutionStage stage) {
        return toString(stage);
    }

    static std::string extractScope(const std::string& full_name) {
        const auto pos = full_name.find('.');
        if (pos == std::string::npos) {
            return "";
        }
        return full_name.substr(0, pos + 1);
    }

    static ComponentPackageRole expectedRoleForPlacement(const std::string& placement) {
        if (placement == "environment") {
            return ComponentPackageRole::Environment;
        }
        if (placement == "form") {
            return ComponentPackageRole::Form;
        }
        if (placement == "interaction") {
            return ComponentPackageRole::Interaction;
        }
        if (placement == "vehicle.common") {
            return ComponentPackageRole::VehicleCommon;
        }
        if (placement == "vehicle.input") {
            return ComponentPackageRole::VehicleInput;
        }
        if (placement == "vehicle.process") {
            return ComponentPackageRole::VehicleProcess;
        }
        if (placement == "vehicle.output") {
            return ComponentPackageRole::VehicleOutput;
        }
        return ComponentPackageRole::Unknown;
    }

    static ExecutionStage expectedStageForPlacement(const std::string& placement) {
        if (placement == "environment") {
            return ExecutionStage::Environment;
        }
        if (placement == "form") {
            return ExecutionStage::Form;
        }
        if (placement == "interaction") {
            return ExecutionStage::Interaction;
        }
        if (placement == "vehicle.input") {
            return ExecutionStage::VehicleInput;
        }
        if (placement == "vehicle.process") {
            return ExecutionStage::VehicleProcess;
        }
        if (placement == "vehicle.output") {
            return ExecutionStage::VehicleOutput;
        }
        return ExecutionStage::None;
    }

    static std::string joinDiagnosticLines(const std::vector<std::string>& values) {
        std::string result;
        for (size_t i = 0; i < values.size(); ++i) {
            result += (i == 0 ? "  - " : "\n  - ");
            result += values[i];
        }
        return result.empty() ? "  - (none)" : result;
    }

    static void validateAssemblyDescriptors(const std::vector<AssemblyDescriptor>& descriptors,
                                            const std::string& selected_form_family,
                                            Result& result) {
        bool has_form_component = false;

        for (const auto& descriptor : descriptors) {
            if (descriptor.placement == "form") {
                has_form_component = true;
            }

            const auto expected_role = expectedRoleForPlacement(descriptor.placement);
            if (expected_role != ComponentPackageRole::Unknown &&
                descriptor.package_role != ComponentPackageRole::Unknown &&
                descriptor.package_role != expected_role) {
                result.errors.push_back(
                    "Component '" + descriptor.name + "' of type '" + descriptor.type_name +
                    "' is registered as role '" + roleLabel(descriptor.package_role) +
                    "' but was assembled in placement '" + descriptor.placement + "'.");
            }

            const auto expected_stage = expectedStageForPlacement(descriptor.placement);
            if (expected_stage == ExecutionStage::None &&
                descriptor.execution_stage != ExecutionStage::None) {
                result.errors.push_back(
                    "Component '" + descriptor.name + "' of type '" + descriptor.type_name +
                    "' is registered for execution stage '" +
                    stageLabel(descriptor.execution_stage) +
                    "' but was assembled in placement '" + descriptor.placement +
                    "' which is non-scheduled and requires execution stage 'none'.");
            } else if (expected_stage != ExecutionStage::None &&
                       descriptor.execution_stage != ExecutionStage::None &&
                       descriptor.execution_stage != expected_stage) {
                result.errors.push_back(
                    "Component '" + descriptor.name + "' of type '" + descriptor.type_name +
                    "' is registered for execution stage '" +
                    stageLabel(descriptor.execution_stage) +
                    "' but was assembled in placement '" + descriptor.placement +
                    "' which runs at stage '" + stageLabel(expected_stage) + "'.");
            }

            const std::string expected_form_family =
                descriptor.selected_form_family.empty()
                    ? selected_form_family
                    : descriptor.selected_form_family;
            if (!expected_form_family.empty() && !descriptor.form_family.empty() &&
                descriptor.form_family != expected_form_family &&
                descriptor.placement != "environment") {
                result.errors.push_back(
                    "Component '" + descriptor.name + "' of type '" + descriptor.type_name +
                    "' targets form family '" + descriptor.form_family +
                    "' but the selected form family is '" + expected_form_family + "'.");
            }
        }

        if (descriptors.empty()) {
            return;
        }

        if (!has_form_component) {
            result.errors.push_back(
                "Mission assembly did not register any components in the 'form' placement.");
        }
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
