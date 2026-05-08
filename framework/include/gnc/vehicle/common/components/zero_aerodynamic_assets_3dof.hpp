#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/common/interfaces/i_aerodynamic_assets_3dof.hpp"

namespace gnc::vehicle::common {

class ZeroAerodynamicAssets3Dof final
    : public gnc::core::ComponentBase,
      public IAerodynamicAssets3Dof,
      public gnc::interfaces::IObservable {
public:
    ZeroAerodynamicAssets3Dof()
        : ComponentBase("ZeroAerodynamicAssets3Dof") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader(config, config_path).validateNoUnknownKeys();
    }

    void update(double) override {}

    gnc::vehicle::output::AeroCoefficients sampleAeroCoefficients3Dof(
        const AeroQuery3Dof& query) const override {
        last_query_ = query;
        ++sample_count_;
        return {};
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("sample_count",
                          [this]() { return static_cast<double>(sample_count_); });
        builder.addScalar("query_angle_of_attack_rad",
                          [this]() { return last_query_.angle_of_attack_rad; });
        builder.addScalar("query_sideslip_angle_rad",
                          [this]() { return last_query_.sideslip_angle_rad; });
        builder.addScalar("query_mach_number",
                          [this]() { return last_query_.mach_number; });
        return builder.build();
    }

private:
    mutable int sample_count_ = 0;
    mutable AeroQuery3Dof last_query_{};
};

} // namespace gnc::vehicle::common
