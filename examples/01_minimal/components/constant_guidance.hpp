#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/interfaces/gnc/i_guidance_6dof.hpp"

class ConstantGuidance : public gnc::core::ComponentBase,
                         public gnc::interfaces::IGuidance6DOF {
public:
    ConstantGuidance() : ComponentBase("ConstantGuidance") {
        setExecutionFrequency(50.0);
    }

    void configure(const gnc::core::ConfigNode& config) override {
        thrust_x_ = config["thrust_x"].asDouble(0.0);
        thrust_y_ = config["thrust_y"].asDouble(0.0);
        thrust_z_ = config["thrust_z"].asDouble(20.0);
    }

    void update(double dt) override {
        (void)dt;
        cmd_.acceleration_cmd = gnc::Vector3d{thrust_x_, thrust_y_, thrust_z_};
        cmd_.timestamp = getSimTime();
    }

    const gnc::interfaces::GuidanceCommand6DOF& getGuidanceCommand() const override {
        return cmd_;
    }

    void setTarget(const gnc::Vector3d& target) override {
        (void)target;
    }

    bool isActive() const override {
        return true;
    }

private:
    gnc::interfaces::GuidanceCommand6DOF cmd_;
    double thrust_x_ = 0.0;
    double thrust_y_ = 0.0;
    double thrust_z_ = 20.0;
};

GNC_REGISTER_COMPONENT(ConstantGuidance, gnc::interfaces::IGuidance6DOF)
