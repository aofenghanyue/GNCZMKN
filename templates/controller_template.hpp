/**
 * @file my_controller.hpp
 * @brief Controller component template.
 *
 * Usage:
 * 1. Copy this file into `user/<project>/components/`.
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
#include "gnc/interfaces/gnc/i_controller.hpp"
#include "gnc/interfaces/gnc/i_guidance.hpp"
#include "gnc/interfaces/gnc/i_navigation.hpp"
#include "gnc/interfaces/infrastructure/i_observable.hpp"

class MyController : public gnc::core::ComponentBase,
                     public gnc::interfaces::IController {
public:
    MyController() : ComponentBase("MyController") {
        setExecutionFrequency(50.0);
    }

    void configure(const gnc::core::ConfigNode& config) override {
        (void)config;
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(
            gnc::core::bind(guidance_, "guidance"),
            gnc::core::bindIfPresent(nav_, "nav"));
    }

    void update(double) override {
        if (!guidance_ || !guidance_->isActive()) {
            return;
        }

        const auto& guidance_command = guidance_->getGuidanceCommand();
        ctrl_cmd_.force_cmd = guidance_command.acceleration_cmd;
        ctrl_cmd_.timestamp = (nav_ && nav_->isValid()) ? nav_->getTimestamp() : getSimTime();
    }

    const gnc::interfaces::ControlCommand& getControlCommand() const override {
        return ctrl_cmd_;
    }

    const gnc::interfaces::ActuatorCommand& getActuatorCommand() const override {
        return act_cmd_;
    }

    bool isActive() const override {
        return true;
    }

private:
    gnc::interfaces::IGuidance* guidance_ = nullptr;
    gnc::interfaces::INavigation* nav_ = nullptr;
    gnc::interfaces::ControlCommand ctrl_cmd_;
    gnc::interfaces::ActuatorCommand act_cmd_;
};

// Optional: add `public gnc::interfaces::IObservable` to the class declaration
// and expose stable output fields if this component should appear in `outputs.record`.
// Add `public gnc::core::IDependencyDeclarer` only when you want the template
// to carry hand-written dependency semantics in addition to real `bind(...)`
// preflight checks from `injectDependencies()`.
//
// std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
//     gnc::core::ObservableFieldBuilder builder;
//     builder.addVector3d("force_cmd",
//         [this]() -> const gnc::Vector3d& { return ctrl_cmd_.force_cmd; });
//     builder.addVector3d("torque_cmd",
//         [this]() -> const gnc::Vector3d& { return ctrl_cmd_.torque_cmd; });
//     builder.addScalar("timestamp", [this]() { return ctrl_cmd_.timestamp; });
//     return builder.build();
// }

GNC_REGISTER_COMPONENT(MyController, gnc::interfaces::IController)
