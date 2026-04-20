#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/plugins/mass/interfaces/i_constant_mass.hpp"

namespace gnc::plugins::cavh {

class ConstantMass final : public gnc::core::ComponentBase,
                           public gnc::plugins::mass::IConstantMass,
                           public gnc::interfaces::IObservable {
public:
    ConstantMass() : ComponentBase("CavhConstantMass") {}

    void configure(const gnc::core::ConfigNode& config) override {
        mass_kg_ = config["mass_kg"].asDouble(mass_kg_);
    }

    void update(double) override {}

    double getMassKg() const override { return mass_kg_; }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("mass_kg", [this]() { return mass_kg_; });
        return builder.build();
    }

private:
    double mass_kg_ = 2000.0 * 0.45359237;
};

} // namespace gnc::plugins::cavh
