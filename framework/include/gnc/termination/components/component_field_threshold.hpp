#pragma once

#include "gnc/common/string_utils.hpp"
#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/interfaces/i_continuous_system.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/interfaces/i_termination_evaluator.hpp"

#include <algorithm>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace gnc::termination {

enum class ComponentFieldThresholdMode {
    Below,
    Above
};

class ComponentFieldThreshold : public gnc::core::ComponentBase,
                                public gnc::interfaces::ITerminationEvaluator {
public:
    ComponentFieldThreshold(std::string name, ComponentFieldThresholdMode mode)
        : ComponentBase(std::move(name)), mode_(mode) {}

    void configure(const gnc::core::ConfigNode& config) override {
        configure(config, "config");
    }

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        component_name_ = reader.requiredString("component");
        field_name_ = reader.requiredString("field");
        threshold_ = reader.requiredDouble("value");
        description_ = reader.optionalString(
            "description",
            modeLabel() + "(" + component_name_ + "." + field_name_ + ", " +
                std::to_string(threshold_) + ")");
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        auto* component =
            registry.getRegistry().get<gnc::core::ComponentBase>(component_name_);
        if (!component) {
            std::string message =
                "Termination component references unknown component '" +
                component_name_ + "'.";
            const std::string suggestion =
                gnc::common::findClosestMatch(component_name_,
                                              registry.getRegistry().getComponentNames());
            if (!suggestion.empty()) {
                message += " Did you mean '" + suggestion + "'?";
            }
            throw std::runtime_error(message);
        }

        auto* observable =
            dynamic_cast<gnc::interfaces::IObservable*>(component);
        auto* continuous_system =
            dynamic_cast<gnc::interfaces::IContinuousSystem*>(component);

        std::vector<std::string> candidate_fields;
        getter_ = {};

        if (observable) {
            const auto fields = observable->getObservableFields();
            candidate_fields.reserve(fields.size());
            for (const auto& field : fields) {
                candidate_fields.push_back(field.name);
                if (!getter_ && field.name == field_name_) {
                    getter_ = field.getter;
                }
            }
        }

        if (!getter_ && continuous_system &&
            continuous_system->getStateLayout().has(field_name_)) {
            getter_ = [continuous_system, field_name = field_name_]() {
                return continuous_system->getStateValue(field_name);
            };
        }

        if (!getter_ && continuous_system) {
            const auto& state_names = continuous_system->getStateLayout().names();
            candidate_fields.insert(candidate_fields.end(),
                                    state_names.begin(),
                                    state_names.end());
        }

        if (!getter_) {
            std::sort(candidate_fields.begin(), candidate_fields.end());
            candidate_fields.erase(std::unique(candidate_fields.begin(),
                                               candidate_fields.end()),
                                   candidate_fields.end());

            std::string message = "Termination component references field '" +
                                  field_name_ + "' not found in component '" +
                                  component_name_ + "'.";
            if (!candidate_fields.empty()) {
                message += " Available fields: " + joinStrings(candidate_fields) + ".";
            }
            const std::string suggestion =
                gnc::common::findClosestMatch(field_name_, candidate_fields);
            if (!suggestion.empty()) {
                message += " Did you mean '" + suggestion + "'?";
            }
            throw std::runtime_error(message);
        }
    }

    void update(double) override {}

    bool shouldTerminate() const override {
        if (!getter_) {
            return false;
        }
        const double value = getter_();
        if (mode_ == ComponentFieldThresholdMode::Below) {
            return value < threshold_;
        }
        return value > threshold_;
    }

    std::string reason() const override { return description_; }

private:
    std::string modeLabel() const {
        return mode_ == ComponentFieldThresholdMode::Below
                   ? "component_field_below"
                   : "component_field_above";
    }

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

    ComponentFieldThresholdMode mode_;
    std::string component_name_;
    std::string field_name_;
    std::string description_;
    double threshold_ = 0.0;
    std::function<double()> getter_;
};

class ComponentFieldBelow final : public ComponentFieldThreshold {
public:
    ComponentFieldBelow()
        : ComponentFieldThreshold("ComponentFieldBelow",
                                  ComponentFieldThresholdMode::Below) {}
};

class ComponentFieldAbove final : public ComponentFieldThreshold {
public:
    ComponentFieldAbove()
        : ComponentFieldThreshold("ComponentFieldAbove",
                                  ComponentFieldThresholdMode::Above) {}
};

} // namespace gnc::termination
