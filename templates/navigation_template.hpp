/**
 * @file my_navigation.hpp
 * @brief Navigation component template.
 *
 * Usage:
 * 1. Copy this file into `user/components/navigation/`.
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
#include "gnc/interfaces/gnc/i_navigation.hpp"
#include "gnc/interfaces/infrastructure/i_observable.hpp"
#include "gnc/interfaces/sensors/i_imu_sensor.hpp"

class MyNavigation : public gnc::core::ComponentBase,
                     public gnc::interfaces::INavigation {
public:
    MyNavigation() : ComponentBase("MyNavigation") {
        setExecutionFrequency(100.0);
    }

    void configure(const gnc::core::ConfigNode& config) override {
        (void)config;
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(gnc::core::bind(imu_, "imu"));
    }

    void update(double dt) override {
        if (!imu_) {
            return;
        }

        const auto& imu_data = imu_->getImuData();
        nav_state_.velocity += imu_data.acceleration * dt;
        nav_state_.position += nav_state_.velocity * dt;
        nav_state_.angular_velocity = imu_data.angular_velocity;
        nav_state_.timestamp = getSimTime();
    }

    const gnc::interfaces::NavState& getNavState() const override {
        return nav_state_;
    }

    double getTimestamp() const override {
        return nav_state_.timestamp;
    }

    bool isValid() const override {
        return true;
    }

private:
    gnc::interfaces::IImuSensor* imu_ = nullptr;
    gnc::interfaces::NavState nav_state_;
};

// Optional: add `public gnc::interfaces::IObservable` to the class declaration
// and expose stable output fields if this component should appear in `outputs.record`.
// `bind(...)` preflight already validates the real required bindings. Add
// `public gnc::core::IDependencyDeclarer` only when you want extra semantic
// descriptions beyond those real binding diagnostics.
//
// std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
//     gnc::core::ObservableFieldBuilder builder;
//     builder.addVector3d("position",
//         [this]() -> const gnc::Vector3d& { return nav_state_.position; });
//     builder.addVector3d("velocity",
//         [this]() -> const gnc::Vector3d& { return nav_state_.velocity; });
//     builder.addScalar("timestamp", [this]() { return nav_state_.timestamp; });
//     return builder.build();
// }

GNC_REGISTER_COMPONENT(MyNavigation, gnc::interfaces::INavigation)
