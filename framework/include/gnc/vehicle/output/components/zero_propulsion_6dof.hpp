#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/output/interfaces/i_propulsion_6dof.hpp"

namespace gnc::vehicle::output {

class ZeroPropulsion6Dof final : public gnc::core::ComponentBase,
                                 public IPropulsion6Dof,
                                 public gnc::interfaces::IObservable {
public:
    ZeroPropulsion6Dof() : ComponentBase("ZeroPropulsion6Dof") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader(config, config_path).validateNoUnknownKeys();
    }

    void update(double) override {}

    gnc::vehicle::ForceMoment6Dof propulsionForceMoment6Dof() const override {
        return force_moment_;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addVector3("force_body", [this]() -> const gnc::math::Vector3& {
            return force_moment_.force_body_n;
        });
        builder.addVector3("moment_body", [this]() -> const gnc::math::Vector3& {
            return force_moment_.moment_body_nm;
        });
        return builder.build();
    }

private:
    gnc::vehicle::ForceMoment6Dof force_moment_{};
};

} // namespace gnc::vehicle::output
