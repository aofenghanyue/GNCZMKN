#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/process/interfaces/i_phase_sequencer_6dof.hpp"

namespace gnc::vehicle::process {

class IdealPhaseSequencer6Dof final : public gnc::core::ComponentBase,
                                      public IPhaseSequencer6Dof,
                                      public gnc::interfaces::IObservable {
public:
    IdealPhaseSequencer6Dof() : ComponentBase("IdealPhaseSequencer6Dof") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        state_.phase_name = reader.optionalString("phase_name", state_.phase_name);
        reader.validateNoUnknownKeys();
    }

    void initialize() override { update(0.0); }
    void update(double) override { state_.elapsed_time_s = getSimTime(); }

    const PhaseState6Dof& phaseState6Dof() const override { return state_; }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("elapsed_time_s",
                          [this]() { return state_.elapsed_time_s; });
        return builder.build();
    }

private:
    PhaseState6Dof state_{};
};

} // namespace gnc::vehicle::process
