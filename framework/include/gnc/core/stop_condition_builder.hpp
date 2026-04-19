#pragma once

#include "gnc/common/string_utils.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/core/simulator.hpp"
#include "gnc/interfaces/i_continuous_system.hpp"
#include "gnc/interfaces/i_observable.hpp"

#include <algorithm>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace gnc::core {

class StopConditionBuilder {
public:
    using DiagnosticReporter = std::function<void(const std::string&)>;

    StopConditionBuilder(Simulator& simulator, DiagnosticReporter add_warning)
        : simulator_(simulator), add_warning_(std::move(add_warning)) {}

    bool build(const ConfigNode& conditions) {
        if (conditions.isNull()) {
            return true;
        }
        if (!conditions.isArray()) {
            add_warning_("simulation.stop_conditions must be an array.");
            return false;
        }

        auto& registry = simulator_.getRegistry();
        bool success = true;

        for (size_t i = 0; i < conditions.size(); ++i) {
            const auto& condition = conditions[i];
            const std::string type = condition["type"].asString();
            const std::string component_name = condition["component"].asString();
            const std::string field_name = condition["field"].asString();
            const double threshold = condition["value"].asDouble(0.0);
            const std::string description = condition["description"].asString(
                type + "(" + component_name + "." + field_name + ", " +
                std::to_string(threshold) + ")");

            if (type.empty() || component_name.empty() || field_name.empty()) {
                add_warning_("Stop condition at index " + std::to_string(i) +
                             " is missing required fields.");
                success = false;
                continue;
            }

            auto* component = registry.get<ComponentBase>(component_name);
            if (!component) {
                std::string message = "Stop condition references unknown component '" +
                                      component_name + "'.";
                const std::string suggestion =
                    common::findClosestMatch(component_name, registry.getComponentNames());
                if (!suggestion.empty()) {
                    message += " Did you mean '" + suggestion + "'?";
                }
                add_warning_(message);
                success = false;
                continue;
            }

            auto* observable = dynamic_cast<interfaces::IObservable*>(component);
            auto* continuous_system =
                dynamic_cast<interfaces::IContinuousSystem*>(component);

            std::function<double()> getter;
            std::vector<std::string> candidate_fields;

            if (observable) {
                const auto fields = observable->getObservableFields();
                candidate_fields.reserve(fields.size());
                for (const auto& field : fields) {
                    candidate_fields.push_back(field.name);
                    if (!getter && field.name == field_name) {
                        getter = field.getter;
                    }
                }
            }

            if (!getter && continuous_system &&
                continuous_system->getStateLayout().has(field_name)) {
                getter = [continuous_system, field_name]() {
                    return continuous_system->getStateValue(field_name);
                };
            }

            if (!getter && continuous_system) {
                const auto& state_names = continuous_system->getStateLayout().names();
                candidate_fields.insert(candidate_fields.end(),
                                        state_names.begin(),
                                        state_names.end());
            }

            if (!getter) {
                std::sort(candidate_fields.begin(), candidate_fields.end());
                candidate_fields.erase(std::unique(candidate_fields.begin(),
                                                  candidate_fields.end()),
                                       candidate_fields.end());

                std::string message = "Stop condition references field '" + field_name +
                                      "' not found in component '" +
                                      component_name + "'.";
                if (observable) {
                    message += " Available observable fields: " +
                               listFieldNames(observable->getObservableFields()) + ".";
                }
                if (continuous_system) {
                    message += " Available state fields: " +
                               listStateFieldNames(continuous_system) + ".";
                }
                const std::string suggestion =
                    common::findClosestMatch(field_name, candidate_fields);
                if (!suggestion.empty()) {
                    message += " Did you mean '" + suggestion + "'?";
                }
                add_warning_(message);
                success = false;
                continue;
            }

            if (type == "component_field_below") {
                simulator_.addTerminationCondition(
                    description,
                    [getter, threshold](int, double) { return getter() < threshold; });
            } else if (type == "component_field_above") {
                simulator_.addTerminationCondition(
                    description,
                    [getter, threshold](int, double) { return getter() > threshold; });
            } else {
                add_warning_("Unknown stop condition type '" + type + "'.");
                success = false;
            }
        }

        return success;
    }

private:
    static std::string joinStrings(const std::vector<std::string>& values) {
        std::string result;
        for (size_t i = 0; i < values.size(); ++i) {
            if (i > 0) {
                result += ", ";
            }
            result += values[i];
        }
        return result.empty() ? "(none)" : result;
    }

    static std::string listFieldNames(
        const std::vector<interfaces::ObservableField>& fields) {
        std::vector<std::string> names;
        names.reserve(fields.size());
        for (const auto& field : fields) {
            names.push_back(field.name);
        }
        return joinStrings(names);
    }

    static std::string listStateFieldNames(
        const interfaces::IContinuousSystem* continuous_system) {
        if (!continuous_system) {
            return "(none)";
        }
        return joinStrings(continuous_system->getStateLayout().names());
    }

    Simulator& simulator_;
    DiagnosticReporter add_warning_;
};

} // namespace gnc::core
