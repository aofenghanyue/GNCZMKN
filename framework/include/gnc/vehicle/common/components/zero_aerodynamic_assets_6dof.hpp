#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/common/interfaces/i_aerodynamic_assets_6dof.hpp"

namespace gnc::vehicle::common {

class ZeroAerodynamicAssets6Dof final : public gnc::core::ComponentBase,
                                        public IAerodynamicAssets6Dof,
                                        public gnc::interfaces::IObservable {
public:
    ZeroAerodynamicAssets6Dof() : ComponentBase("ZeroAerodynamicAssets6Dof") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader(config, config_path).validateNoUnknownKeys();
    }

    void update(double) override {}

    gnc::vehicle::AeroCoefficients6Dof sampleAeroCoefficients6Dof(
        const gnc::vehicle::AeroQuery6Dof& query) const override {
        last_query_ = query;
        ++sample_count_;
        return {};
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("sample_count",
                          [this]() { return static_cast<double>(sample_count_); });
        builder.addScalar("last_query.alpha_rad",
                          [this]() { return last_query_.alpha_rad; });
        builder.addScalar("last_query.beta_rad",
                          [this]() { return last_query_.beta_rad; });
        builder.addScalar("last_query.mach_number",
                          [this]() { return last_query_.mach_number; });
        return builder.build();
    }

private:
    mutable int sample_count_ = 0;
    mutable gnc::vehicle::AeroQuery6Dof last_query_{};
};

} // namespace gnc::vehicle::common
