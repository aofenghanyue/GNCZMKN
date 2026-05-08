#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/output/interfaces/i_mass_properties_6dof.hpp"

namespace gnc::vehicle::output {

class ConstantMassProperties6Dof final : public gnc::core::ComponentBase,
                                        public IMassProperties6Dof,
                                        public gnc::interfaces::IObservable {
public:
    ConstantMassProperties6Dof()
        : ComponentBase("ConstantMassProperties6Dof") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        properties_.mass_kg = reader.requiredDouble("mass_kg");
        const auto cg = reader.requiredDoubleArray("center_of_gravity_body_m", 3);
        const auto inertia = reader.requiredDoubleArray("inertia_body_kgm2", 3);
        properties_.center_of_gravity_body_m =
            gnc::math::Vector3(cg[0], cg[1], cg[2]);
        properties_.inertia_body_kgm2 = gnc::math::Matrix3::Zero();
        properties_.inertia_body_kgm2(0, 0) = inertia[0];
        properties_.inertia_body_kgm2(1, 1) = inertia[1];
        properties_.inertia_body_kgm2(2, 2) = inertia[2];
        reader.validateNoUnknownKeys();
    }

    void update(double) override {}

    gnc::vehicle::MassProperties6Dof massProperties6Dof() const override {
        return properties_;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("mass_kg", [this]() { return properties_.mass_kg; });
        builder.addVector3("center_of_gravity_body",
                           [this]() -> const gnc::math::Vector3& {
                               return properties_.center_of_gravity_body_m;
                           });
        builder.addScalar("inertia_body.ixx",
                          [this]() { return properties_.inertia_body_kgm2(0, 0); });
        builder.addScalar("inertia_body.iyy",
                          [this]() { return properties_.inertia_body_kgm2(1, 1); });
        builder.addScalar("inertia_body.izz",
                          [this]() { return properties_.inertia_body_kgm2(2, 2); });
        return builder.build();
    }

private:
    gnc::vehicle::MassProperties6Dof properties_{};
};

} // namespace gnc::vehicle::output
