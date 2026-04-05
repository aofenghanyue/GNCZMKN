#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/core/observable_helpers.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/interfaces/data_types.hpp"
#include "gnc/interfaces/gnc/i_guidance.hpp"
#include "gnc/interfaces/gnc/i_navigation.hpp"
#include "gnc/interfaces/infrastructure/i_observable.hpp"

class ConstantGuidance : public gnc::core::ComponentBase,
                         public gnc::interfaces::IGuidance,
                         public gnc::interfaces::IObservable {
public:
    ConstantGuidance() : ComponentBase("ConstantGuidance") {
        setExecutionFrequency(50.0);
    }

    void configure(const gnc::core::ConfigNode& config) override {
        thrust_x_ = config["thrust_x"].asDouble(0.0);
        thrust_y_ = config["thrust_y"].asDouble(0.0);
        thrust_z_ = config["thrust_z"].asDouble(20.0);
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        nav_ = registry.getByName<gnc::interfaces::INavigation>("nav");
    }

    void update(double dt) override {
        (void)dt;
        cmd_.acceleration_cmd = gnc::Vector3d{thrust_x_, thrust_y_, thrust_z_};
        cmd_.timestamp = nav_ ? nav_->getTimestamp() : getSimTime();
    }

    const gnc::interfaces::GuidanceCommand& getGuidanceCommand() const override {
        return cmd_;
    }

    void setTarget(const gnc::Vector3d& target) override {
        (void)target;
    }

    bool isActive() const override {
        return true;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addVector3d("acceleration_cmd", [this]() -> const gnc::Vector3d& { return cmd_.acceleration_cmd; });
        builder.addScalar("timestamp", [this]() { return cmd_.timestamp; });
        return builder.build();
    }

private:
    gnc::interfaces::INavigation* nav_ = nullptr;
    gnc::interfaces::GuidanceCommand cmd_;
    double thrust_x_ = 0.0;
    double thrust_y_ = 0.0;
    double thrust_z_ = 20.0;
};

GNC_REGISTER_COMPONENT(ConstantGuidance, gnc::interfaces::IGuidance)
