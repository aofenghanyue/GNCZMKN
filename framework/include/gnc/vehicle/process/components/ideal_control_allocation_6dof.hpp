#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/process/interfaces/i_attitude_control_6dof.hpp"
#include "gnc/vehicle/process/interfaces/i_control_allocation_6dof.hpp"

namespace gnc::vehicle::process {

class IdealControlAllocation6Dof final : public gnc::core::ComponentBase,
                                        public IControlAllocation6Dof,
                                        public gnc::interfaces::IObservable {
public:
    IdealControlAllocation6Dof()
        : ComponentBase("IdealControlAllocation6Dof") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        attitude_control_lookup_name_ =
            reader.optionalString("attitude_control_lookup_name",
                                  attitude_control_lookup_name_);
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(gnc::core::bindIfPresent(
            attitude_control_, attitude_control_lookup_name_));
    }

    void initialize() override { update(0.0); }

    void update(double) override {
        (void)attitude_control_;
        command_.control_surfaces.fin_deflection_rad = gnc::math::Vector3::Zero();
    }

    const ControlAllocationCommand6Dof& controlAllocationCommand6Dof()
        const override {
        return command_;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addVector3("fin_deflection",
                           [this]() -> const gnc::math::Vector3& {
                               return command_.control_surfaces.fin_deflection_rad;
                           });
        return builder.build();
    }

private:
    IAttitudeControl6Dof* attitude_control_ = nullptr;
    std::string attitude_control_lookup_name_ = "attitude_control";
    ControlAllocationCommand6Dof command_{};
};

} // namespace gnc::vehicle::process
