/**
 * @file my_guidance.hpp
 * @brief Guidance component template.
 *
 * Usage:
 * 1. Copy this file into `user/components/guidance/`.
 * 2. Rename the file and class.
 * 3. Replace the placeholder logic in `configure()` and `update()`.
 * 4. Rebuild the project.
 * 5. Use the class name as the mission `type`.
 */
#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/core/observable_helpers.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/interfaces/gnc/i_guidance.hpp"
#include "gnc/interfaces/gnc/i_navigation.hpp"
#include "gnc/interfaces/infrastructure/i_observable.hpp"

class MyGuidance : public gnc::core::ComponentBase,
                   public gnc::interfaces::IGuidance {
public:
    MyGuidance() : ComponentBase("MyGuidance") {
        setExecutionFrequency(50.0);
    }

    void configure(const gnc::core::ConfigNode& config) override {
        (void)config;
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(gnc::core::bind(nav_, "nav"));
    }

    void update(double dt) override {
        (void)dt;
        if (!nav_ || !nav_->isValid()) {
            return;
        }

        const auto& state = nav_->getNavState();
        cmd_.acceleration_cmd = gnc::Vector3d{0.0, 0.0, 0.0};
        cmd_.timestamp = state.timestamp;
    }

    const gnc::interfaces::GuidanceCommand& getGuidanceCommand() const override {
        return cmd_;
    }

    void setTarget(const gnc::Vector3d& target) override {
        target_ = target;
    }

    bool isActive() const override {
        return true;
    }

private:
    gnc::interfaces::INavigation* nav_ = nullptr;
    gnc::interfaces::GuidanceCommand cmd_;
    gnc::Vector3d target_;
};

// Optional: add `public gnc::interfaces::IObservable` to the class declaration
// and expose stable output fields if this component should appear in `outputs.record`.
// `bind(...)` preflight already checks the real required bindings and can aggregate
// multiple failures from `injectDependencies()`. Add
// `public gnc::core::IDependencyDeclarer` only when you also want hand-written
// semantic descriptions on top of those real binding diagnostics.
//
// std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
//     gnc::core::ObservableFieldBuilder builder;
//     builder.addVector3d("acceleration_cmd",
//         [this]() -> const gnc::Vector3d& { return cmd_.acceleration_cmd; });
//     builder.addScalar("timestamp", [this]() { return cmd_.timestamp; });
//     return builder.build();
// }

GNC_REGISTER_COMPONENT(MyGuidance, gnc::interfaces::IGuidance)
