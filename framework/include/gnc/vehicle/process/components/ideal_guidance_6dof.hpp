#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/process/interfaces/i_guidance_6dof.hpp"
#include "gnc/vehicle/process/interfaces/i_target_tracking_6dof.hpp"

namespace gnc::vehicle::process {

class IdealGuidance6Dof final : public gnc::core::ComponentBase,
                                public IGuidance6Dof,
                                public gnc::interfaces::IObservable {
public:
    IdealGuidance6Dof() : ComponentBase("IdealGuidance6Dof") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        tracking_lookup_name_ =
            reader.optionalString("tracking_lookup_name", tracking_lookup_name_);
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(gnc::core::bind(tracking_, tracking_lookup_name_));
    }

    void initialize() override { update(0.0); }

    void update(double) override {
        command_.line_of_sight_ecef =
            tracking_->targetTrack6Dof().line_of_sight_ecef;
        command_.acceleration_command_nue_mps2 = gnc::math::Vector3::Zero();
    }

    const GuidanceCommand6Dof& guidanceCommand6Dof() const override {
        return command_;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addVector3("line_of_sight_ecef",
                           [this]() -> const gnc::math::Vector3& {
                               return command_.line_of_sight_ecef;
                           });
        builder.addVector3("acceleration_command_nue",
                           [this]() -> const gnc::math::Vector3& {
                               return command_.acceleration_command_nue_mps2;
                           });
        return builder.build();
    }

private:
    ITargetTracking6Dof* tracking_ = nullptr;
    std::string tracking_lookup_name_ = "tracking";
    GuidanceCommand6Dof command_{};
};

} // namespace gnc::vehicle::process
