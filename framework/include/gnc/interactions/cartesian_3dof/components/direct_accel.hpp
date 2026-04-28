#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/forms/cartesian_3dof/interfaces/i_input_provider.hpp"

#include <string>

namespace gnc::interactions::cartesian_3dof {

class DirectAccel final : public gnc::core::ComponentBase,
                          public gnc::forms::cartesian_3dof::IInputProvider {
public:
    DirectAccel() : ComponentBase("Cartesian3DoFDirectAccel") {}

    void configure(const gnc::core::ConfigNode& config) override {
        configure(config, "config");
    }

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        std::vector<double> acceleration;
        if (reader.has("acceleration_mps2")) {
            acceleration = reader.requiredDoubleArray("acceleration_mps2", 3);
        } else if (reader.has("constant_acceleration")) {
            acceleration = reader.requiredDoubleArray("constant_acceleration", 3);
        } else if (reader.has("ax_mps2") || reader.has("ay_mps2") ||
                   reader.has("az_mps2")) {
            acceleration = {reader.requiredDouble("ax_mps2"),
                            reader.requiredDouble("ay_mps2"),
                            reader.requiredDouble("az_mps2")};
        } else {
            throw std::runtime_error(config_path +
                                     ".acceleration_mps2 is required and must contain exactly 3 numbers.");
        }

        configured_input_.acceleration_mps2 =
            gnc::math::Vector3(acceleration[0], acceleration[1], acceleration[2]);
        reader.validateNoUnknownKeys();
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
