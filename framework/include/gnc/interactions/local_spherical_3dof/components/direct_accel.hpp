#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_input_provider.hpp"

namespace gnc::interactions::local_spherical_3dof {

class DirectAccel final : public gnc::core::ComponentBase,
                          public gnc::forms::local_spherical_3dof::IInputProvider {
public:
    DirectAccel() : ComponentBase("LocalSpherical3DoFDirectAccel") {}

    void configure(const gnc::core::ConfigNode& config) override {
        const auto& vector = config["local_acceleration_nue_mps2"];
        if (vector.isArray() && vector.size() >= 3) {
            configured_input_.local_acceleration_nue_mps2 = gnc::math::Vector3(
                vector[0].asDouble(0.0),
                vector[1].asDouble(0.0),
                vector[2].asDouble(0.0));
            return;
        }

        configured_input_.local_acceleration_nue_mps2 = gnc::math::Vector3(
            config["tangential_accel_mps2"].asDouble(0.0),
            config["normal_accel_mps2"].asDouble(-9.80665),
            config["lateral_accel_mps2"].asDouble(0.0));
    }

    void update(double) override {}

    gnc::forms::local_spherical_3dof::Input computeLocalSpherical3DoFInput(
        const gnc::forms::local_spherical_3dof::Truth&,
        double) const override {
        return configured_input_;
    }

private:
    gnc::forms::local_spherical_3dof::Input configured_input_{};
};

} // namespace gnc::interactions::local_spherical_3dof
