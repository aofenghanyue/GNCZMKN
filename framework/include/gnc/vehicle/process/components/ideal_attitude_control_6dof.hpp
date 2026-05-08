#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/process/interfaces/i_attitude_control_6dof.hpp"
#include "gnc/vehicle/process/interfaces/i_guidance_6dof.hpp"

namespace gnc::vehicle::process {

class IdealAttitudeControl6Dof final : public gnc::core::ComponentBase,
                                       public IAttitudeControl6Dof,
                                       public gnc::interfaces::IObservable {
public:
    IdealAttitudeControl6Dof()
        : ComponentBase("IdealAttitudeControl6Dof") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        guidance_lookup_name_ =
            reader.optionalString("guidance_lookup_name", guidance_lookup_name_);
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(gnc::core::bindIfPresent(guidance_, guidance_lookup_name_));
    }

    void initialize() override { update(0.0); }

    void update(double) override {
        (void)guidance_;
        command_.body_rate_command_radps = gnc::math::Vector3::Zero();
        command_.moment_command_body_nm = gnc::math::Vector3::Zero();
    }

    const AttitudeControlCommand6Dof& attitudeControlCommand6Dof()
        const override {
        return command_;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addVector3("body_rate_command",
                           [this]() -> const gnc::math::Vector3& {
                               return command_.body_rate_command_radps;
                           });
        builder.addVector3("moment_command_body",
                           [this]() -> const gnc::math::Vector3& {
                               return command_.moment_command_body_nm;
                           });
        return builder.build();
    }

private:
    IGuidance6Dof* guidance_ = nullptr;
    std::string guidance_lookup_name_ = "guidance";
    AttitudeControlCommand6Dof command_{};
};

} // namespace gnc::vehicle::process
