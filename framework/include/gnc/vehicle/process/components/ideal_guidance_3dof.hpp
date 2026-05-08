#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/process/interfaces/i_guidance_3dof.hpp"
#include "gnc/vehicle/process/interfaces/i_target_tracking_3dof.hpp"

namespace gnc::vehicle::process {

class IdealGuidance3Dof final : public gnc::core::ComponentBase,
                                public IGuidance3Dof,
                                public gnc::interfaces::IObservable {
public:
    IdealGuidance3Dof() : ComponentBase("IdealGuidance3Dof") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        tracking_lookup_name_ =
            reader.optionalString("tracking_lookup_name", tracking_lookup_name_);
        command_.angle_of_attack_rad =
            reader.optionalDouble("angle_of_attack_rad",
                                  command_.angle_of_attack_rad);
        command_.bank_angle_rad =
            reader.optionalDouble("bank_angle_rad", command_.bank_angle_rad);
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(gnc::core::bind(tracking_, tracking_lookup_name_));
    }

    void initialize() override { update(0.0); }

    void update(double) override {
        command_.line_of_sight_ecef =
            tracking_->targetTrack3Dof().line_of_sight_ecef;
        command_.acceleration_command_nue_mps2 = gnc::math::Vector3::Zero();
    }

    const GuidanceCommand3Dof& guidanceCommand3Dof() const override {
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
        builder.addScalar("angle_of_attack_rad",
                          [this]() { return command_.angle_of_attack_rad; });
        builder.addScalar("bank_angle_rad",
                          [this]() { return command_.bank_angle_rad; });
        return builder.build();
    }

private:
    ITargetTracking3Dof* tracking_ = nullptr;
    std::string tracking_lookup_name_ = "tracking";
    GuidanceCommand3Dof command_{};
};

} // namespace gnc::vehicle::process
