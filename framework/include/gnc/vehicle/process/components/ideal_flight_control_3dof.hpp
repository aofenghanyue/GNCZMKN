#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/process/interfaces/i_flight_control_3dof.hpp"
#include "gnc/vehicle/process/interfaces/i_guidance_3dof.hpp"

namespace gnc::vehicle::process {

class IdealFlightControl3Dof final : public gnc::core::ComponentBase,
                                     public IFlightControl3Dof,
                                     public gnc::interfaces::IObservable {
public:
    IdealFlightControl3Dof() : ComponentBase("IdealFlightControl3Dof") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        guidance_lookup_name_ =
            reader.optionalString("guidance_lookup_name", guidance_lookup_name_);
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(gnc::core::bind(guidance_, guidance_lookup_name_));
    }

    void initialize() override { update(0.0); }

    void update(double) override {
        const auto& guidance = guidance_->guidanceCommand3Dof();
        command_.angle_of_attack_rad = guidance.angle_of_attack_rad;
        command_.bank_angle_rad = guidance.bank_angle_rad;
    }

    const FlightControlCommand3Dof& flightControlCommand3Dof() const override {
        return command_;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("angle_of_attack_rad",
                          [this]() { return command_.angle_of_attack_rad; });
        builder.addScalar("bank_angle_rad",
                          [this]() { return command_.bank_angle_rad; });
        return builder.build();
    }

private:
    IGuidance3Dof* guidance_ = nullptr;
    std::string guidance_lookup_name_ = "guidance";
    FlightControlCommand3Dof command_{};
};

} // namespace gnc::vehicle::process
