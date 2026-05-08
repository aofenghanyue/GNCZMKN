#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/process/interfaces/i_control_allocation_3dof.hpp"
#include "gnc/vehicle/process/interfaces/i_flight_control_3dof.hpp"

namespace gnc::vehicle::process {

class IdealControlAllocation3Dof final
    : public gnc::core::ComponentBase,
      public IControlAllocation3Dof,
      public gnc::interfaces::IObservable {
public:
    IdealControlAllocation3Dof()
        : ComponentBase("IdealControlAllocation3Dof") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        flight_control_lookup_name_ =
            reader.optionalString("flight_control_lookup_name",
                                  flight_control_lookup_name_);
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(
            gnc::core::bind(flight_control_, flight_control_lookup_name_));
    }

    void initialize() override { update(0.0); }

    void update(double) override {
        const auto& control = flight_control_->flightControlCommand3Dof();
        command_.angle_of_attack_rad = control.angle_of_attack_rad;
        command_.bank_angle_rad = control.bank_angle_rad;
    }

    const ControlAllocationCommand3Dof& controlAllocationCommand3Dof()
        const override {
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
    IFlightControl3Dof* flight_control_ = nullptr;
    std::string flight_control_lookup_name_ = "flight_control";
    ControlAllocationCommand3Dof command_{};
};

} // namespace gnc::vehicle::process
