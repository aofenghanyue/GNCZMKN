#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/common/assets/json_asset_loader.hpp"
#include "gnc/vehicle/output/interfaces/i_constant_mass.hpp"

namespace gnc::vehicle::output {

class ConstantMass final : public gnc::core::ComponentBase,
                           public IConstantMass,
                           public gnc::interfaces::IObservable {
public:
    ConstantMass() : ComponentBase("ConstantMass") {}

    void configure(const gnc::core::ConfigNode& config) override {
        const auto source =
            gnc::vehicle::common::assets::loadConfiguredJsonAsset(config,
                                                                  "mass.constant");
        mass_kg_ = source["mass_kg"].asDouble(mass_kg_);
    }

    void update(double) override {}

    double getMassKg() const override { return mass_kg_; }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("mass_kg", [this]() { return mass_kg_; });
        return builder.build();
    }

private:
    double mass_kg_ = 1000.0;
};

} // namespace gnc::vehicle::output
