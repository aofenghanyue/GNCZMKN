#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_input_provider.hpp"

namespace tests::project_components {

class LocalSphericalAccelerationInput final
    : public gnc::core::ComponentBase,
      public gnc::forms::local_spherical_3dof::IInputProvider {
public:
    LocalSphericalAccelerationInput()
        : ComponentBase("TestLocalSphericalAccelerationInput") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        const auto acceleration =
            reader.optionalDoubleArray("local_acceleration_nue_mps2",
                                       {0.0, 0.0, 0.0},
                                       3);
        input_.local_acceleration_nue_mps2 =
            gnc::math::Vector3(acceleration[0], acceleration[1], acceleration[2]);
        reader.validateNoUnknownKeys();
    }

    void update(double) override {}

    gnc::forms::local_spherical_3dof::Input computeLocalSpherical3DoFInput(
        const gnc::forms::local_spherical_3dof::Truth&,
        double) const override {
        return input_;
    }

private:
    gnc::forms::local_spherical_3dof::Input input_{};
};

} // namespace tests::project_components

GNC_REGISTER_COMPONENT_TYPE(
    "test_fixture.local_spherical_3dof.acceleration_input",
    tests::project_components::LocalSphericalAccelerationInput,
    ::gnc::core::ComponentPackageRole::Interaction,
    ::gnc::core::ExecutionStage::Interaction,
    "local_spherical_3dof",
    ::gnc::forms::local_spherical_3dof::IInputProvider)
