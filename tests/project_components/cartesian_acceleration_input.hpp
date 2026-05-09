#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/forms/cartesian_3dof/interfaces/i_input_provider.hpp"

namespace tests::project_components {

class CartesianAccelerationInput final
    : public gnc::core::ComponentBase,
      public gnc::forms::cartesian_3dof::IInputProvider {
public:
    CartesianAccelerationInput()
        : ComponentBase("TestCartesianAccelerationInput") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        const auto acceleration =
            reader.optionalDoubleArray("acceleration_mps2",
                                       {0.0, 0.0, 0.0},
                                       3);
        input_.acceleration_mps2 =
            gnc::math::Vector3(acceleration[0], acceleration[1], acceleration[2]);
        reader.validateNoUnknownKeys();
    }

    void update(double) override {}

    gnc::forms::cartesian_3dof::Input computeCartesian3DoFInput(
        const gnc::forms::cartesian_3dof::Truth&,
        double) const override {
        return input_;
    }

private:
    gnc::forms::cartesian_3dof::Input input_{};
};

} // namespace tests::project_components

GNC_REGISTER_COMPONENT_TYPE(
    "test_fixture.cartesian_3dof.acceleration_input",
    tests::project_components::CartesianAccelerationInput,
    ::gnc::core::ComponentPackageRole::Interaction,
    ::gnc::core::ExecutionStage::Interaction,
    "cartesian_3dof",
    ::gnc::forms::cartesian_3dof::IInputProvider)
