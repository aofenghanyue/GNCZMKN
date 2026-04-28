#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_input_provider.hpp"

#include <string>

namespace gnc::interactions::local_spherical_3dof {

class DirectAccel final : public gnc::core::ComponentBase,
                          public gnc::forms::local_spherical_3dof::IInputProvider {
public:
    DirectAccel() : ComponentBase("LocalSpherical3DoFDirectAccel") {}

    void configure(const gnc::core::ConfigNode& config) override {
        configure(config, "config");
    }

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        std::vector<double> acceleration;
        if (reader.has("local_acceleration_nue_mps2")) {
            acceleration = reader.requiredDoubleArray("local_acceleration_nue_mps2", 3);
        } else if (reader.has("tangential_accel_mps2") ||
                   reader.has("normal_accel_mps2") ||
                   reader.has("lateral_accel_mps2")) {
            acceleration = {reader.requiredDouble("tangential_accel_mps2"),
                            reader.requiredDouble("normal_accel_mps2"),
                            reader.requiredDouble("lateral_accel_mps2")};
        } else {
            throw std::runtime_error(
                config_path +
                ".local_acceleration_nue_mps2 is required and must contain exactly 3 numbers.");
        }

        configured_input_.local_acceleration_nue_mps2 =
            gnc::math::Vector3(acceleration[0], acceleration[1], acceleration[2]);
        reader.validateNoUnknownKeys();
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
