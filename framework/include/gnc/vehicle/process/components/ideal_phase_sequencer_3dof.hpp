#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/process/interfaces/i_phase_sequencer_3dof.hpp"

namespace gnc::vehicle::process {

class IdealPhaseSequencer3Dof final : public gnc::core::ComponentBase,
                                      public IPhaseSequencer3Dof,
                                      public gnc::interfaces::IObservable {
public:
    IdealPhaseSequencer3Dof() : ComponentBase("IdealPhaseSequencer3Dof") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        state_.phase_name = reader.optionalString("phase_name", state_.phase_name);
        reader.validateNoUnknownKeys();
    }

    void initialize() override { update(0.0); }

    void update(double) override { state_.elapsed_time_s = getSimTime(); }

    const PhaseState3Dof& phaseState3Dof() const override { return state_; }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("elapsed_time_s",
                          [this]() { return state_.elapsed_time_s; });
        return builder.build();
    }

private:
    PhaseState3Dof state_{};
};

} // namespace gnc::vehicle::process
