#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/output/interfaces/i_actuator_6dof.hpp"
#include "gnc/vehicle/process/interfaces/i_control_allocation_6dof.hpp"

namespace gnc::vehicle::output {

class IdealActuator6Dof final : public gnc::core::ComponentBase,
                                public IActuator6Dof,
                                public gnc::interfaces::IObservable {
public:
    IdealActuator6Dof() : ComponentBase("IdealActuator6Dof") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        control_allocation_lookup_name_ =
            reader.optionalString("control_allocation_lookup_name",
                                  control_allocation_lookup_name_);
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(gnc::core::bindIfPresent(
            control_allocation_, control_allocation_lookup_name_));
    }

    void update(double) override {
        if (control_allocation_) {
            state_ = control_allocation_->controlAllocationCommand6Dof().control_surfaces;
        }
    }

    gnc::vehicle::ControlSurfaceState6Dof actuatorState6Dof() const override {
        return state_;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addVector3("fin_deflection",
                           [this]() -> const gnc::math::Vector3& {
                               return state_.fin_deflection_rad;
                           });
        return builder.build();
    }

private:
    gnc::vehicle::process::IControlAllocation6Dof* control_allocation_ = nullptr;
    std::string control_allocation_lookup_name_ = "control_allocation";
    gnc::vehicle::ControlSurfaceState6Dof state_{};
};

} // namespace gnc::vehicle::output
