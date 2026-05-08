#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/output/interfaces/i_actuator_3dof.hpp"
#include "gnc/vehicle/process/interfaces/i_control_allocation_3dof.hpp"

namespace gnc::vehicle::output {

class IdealActuator3Dof final : public gnc::core::ComponentBase,
                                public IActuator3Dof,
                                public gnc::interfaces::IObservable {
public:
    IdealActuator3Dof() : ComponentBase("IdealActuator3Dof") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        allocation_lookup_name_ =
            reader.optionalString("allocation_lookup_name",
                                  allocation_lookup_name_);
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(
            gnc::core::bindIfPresent(allocation_, allocation_lookup_name_));
    }

    void initialize() override { update(0.0); }

    void update(double) override {
        if (!allocation_) {
            state_ = {};
            return;
        }
        const auto& command = allocation_->controlAllocationCommand3Dof();
        state_.angle_of_attack_rad = command.angle_of_attack_rad;
        state_.bank_angle_rad = command.bank_angle_rad;
    }

    const ActuatorState3Dof& actuatorState3Dof() const override {
        return state_;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("angle_of_attack_rad",
                          [this]() { return state_.angle_of_attack_rad; });
        builder.addScalar("bank_angle_rad",
                          [this]() { return state_.bank_angle_rad; });
        return builder.build();
    }

private:
    gnc::vehicle::process::IControlAllocation3Dof* allocation_ = nullptr;
    std::string allocation_lookup_name_ = "control_allocation";
    ActuatorState3Dof state_{};
};

} // namespace gnc::vehicle::output
