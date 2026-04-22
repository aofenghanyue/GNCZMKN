#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/state_layout.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_continuous_system.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/common/assets/json_asset_loader.hpp"
#include "gnc/vehicle/output/interfaces/i_continuous_mass.hpp"

namespace gnc::vehicle::output {

class ContinuousConstantRateMass final : public gnc::core::ComponentBase,
                                         public gnc::interfaces::IContinuousSystem,
                                         public IContinuousMass,
                                         public gnc::interfaces::IObservable {
public:
    ContinuousConstantRateMass() : ComponentBase("ContinuousConstantRateMass") {
        mass_index_ = layout_.addVariable("mass_kg");
        state_ = Eigen::VectorXd::Zero(layout_.dimension());
        initial_state_ = state_;
    }

    void configure(const gnc::core::ConfigNode& config) override {
        const auto source =
            gnc::vehicle::common::assets::loadConfiguredJsonAsset(
                config,
                "mass.continuous_constant_rate");
        initial_state_[mass_index_] =
            source["initial_mass_kg"].asDouble(900.0);
        mass_rate_kg_per_s_ =
            source["mass_rate_kg_per_s"].asDouble(mass_rate_kg_per_s_);
        state_ = initial_state_;
    }

    const gnc::core::StateLayout& getStateLayout() const override { return layout_; }

    void computeDerivatives(double,
                            const Eigen::VectorXd&,
                            Eigen::VectorXd& derivative) const override {
        derivative = Eigen::VectorXd::Zero(layout_.dimension());
        derivative[mass_index_] = mass_rate_kg_per_s_;
    }

    const Eigen::VectorXd& getState() const override { return state_; }
    void setState(const Eigen::VectorXd& state) override { state_ = state; }
    Eigen::VectorXd getInitialState() const override { return initial_state_; }

    void update(double) override {}

    double getMassKg() const override { return state_[mass_index_]; }
    double getMassRateKgPerSec() const override { return mass_rate_kg_per_s_; }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("mass_kg", [this]() { return getMassKg(); });
        builder.addScalar("mass_rate_kg_per_s",
                          [this]() { return getMassRateKgPerSec(); });
        return builder.build();
    }

private:
    gnc::core::StateLayout layout_;
    Eigen::VectorXd state_;
    Eigen::VectorXd initial_state_;
    double mass_rate_kg_per_s_ = 0.0;
    int mass_index_ = -1;
};

} // namespace gnc::vehicle::output
