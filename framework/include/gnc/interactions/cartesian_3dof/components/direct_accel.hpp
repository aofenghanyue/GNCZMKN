#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/forms/cartesian_3dof/interfaces/i_input_provider.hpp"

namespace gnc::interactions::cartesian_3dof {

class DirectAccel final : public gnc::core::ComponentBase,
                          public gnc::forms::cartesian_3dof::IInputProvider {
public:
    DirectAccel() : ComponentBase("Cartesian3DoFDirectAccel") {}

    void configure(const gnc::core::ConfigNode& config) override {
        const auto& acceleration = config["acceleration_mps2"];
        if (acceleration.isArray() && acceleration.size() >= 3) {
            configured_input_.acceleration_mps2 = gnc::math::Vector3(
                acceleration[0].asDouble(0.0),
                acceleration[1].asDouble(0.0),
                acceleration[2].asDouble(0.0));
            return;
        }

        const auto& legacy_acceleration = config["constant_acceleration"];
        if (legacy_acceleration.isArray() && legacy_acceleration.size() >= 3) {
            configured_input_.acceleration_mps2 = gnc::math::Vector3(
                legacy_acceleration[0].asDouble(0.0),
                legacy_acceleration[1].asDouble(0.0),
                legacy_acceleration[2].asDouble(0.0));
            return;
        }

        configured_input_.acceleration_mps2 = gnc::math::Vector3(
            config["ax_mps2"].asDouble(0.0),
            config["ay_mps2"].asDouble(0.0),
            config["az_mps2"].asDouble(0.0));
    }

    void update(double) override {}

    gnc::forms::cartesian_3dof::Input computeCartesian3DoFInput(
        const gnc::forms::cartesian_3dof::Truth&,
        double) const override {
        return configured_input_;
    }

private:
    gnc::forms::cartesian_3dof::Input configured_input_{};
};

} // namespace gnc::interactions::cartesian_3dof
