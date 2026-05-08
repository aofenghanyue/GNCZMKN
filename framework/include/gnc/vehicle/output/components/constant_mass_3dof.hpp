#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/output/interfaces/i_constant_mass.hpp"

namespace gnc::vehicle::output {

class ConstantMass3Dof final : public gnc::core::ComponentBase,
                               public IConstantMass,
                               public gnc::interfaces::IObservable {
public:
    ConstantMass3Dof() : ComponentBase("ConstantMass3Dof") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        mass_kg_ = reader.requiredDouble("mass_kg");
        reader.validateNoUnknownKeys();
    }

    void update(double) override {}

    double getMassKg() const override { return mass_kg_; }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("mass_kg", [this]() { return mass_kg_; });
        return builder.build();
    }

private:
    double mass_kg_ = 1.0;
};

} // namespace gnc::vehicle::output
