#pragma once

#include "gnc/common/math/eigen_types.hpp"
#include "gnc/core/component_base.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/plugins/state_3dof/interfaces/i_flight_command_provider_3dof.hpp"
#include "gnc/plugins/state_3dof/interfaces/i_state_solver_3dof.hpp"

#include <algorithm>
#include <vector>

namespace gnc::vehicle::process {

class ProgrammedAoAGuidance final
    : public gnc::core::ComponentBase,
      public gnc::plugins::state_3dof::IFlightCommandProvider3DOF,
      public gnc::interfaces::IObservable {
public:
    ProgrammedAoAGuidance() : ComponentBase("ProgrammedAoAGuidance") {
        setExecutionFrequency(20.0);
    }

    void configure(const gnc::core::ConfigNode& config) override {
        bank_angle_deg_ = config["bank_angle_deg"].asDouble(bank_angle_deg_);
        readSchedule(config["schedule_altitude_m"], schedule_altitude_m_);
        readSchedule(config["schedule_angle_of_attack_deg"], schedule_alpha_deg_);

        if (schedule_altitude_m_.empty() ||
            schedule_altitude_m_.size() != schedule_alpha_deg_.size()) {
            schedule_altitude_m_ = {60000.0, 45000.0, 30000.0, 15000.0};
            schedule_alpha_deg_ = {20.0, 12.0, 10.0, 8.0};
        }
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(gnc::core::bind(state_solver_, "dynamics"));
    }

    void update(double) override {
        const double altitude =
            state_solver_ ? state_solver_->getAltitude() : schedule_altitude_m_.front();
        command_.angle_of_attack_rad =
            gnc::math::deg2rad(interpolateAngleOfAttackDeg(altitude));
        command_.bank_angle_rad = gnc::math::deg2rad(bank_angle_deg_);
        command_.timestamp = getSimTime();
    }

    const gnc::plugins::state_3dof::FlightCommand3DOF& getFlightCommand() const override {
        return command_;
    }

    bool isActive() const override { return true; }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("angle_of_attack_deg",
                          [this]() { return gnc::math::rad2deg(command_.angle_of_attack_rad); });
        builder.addScalar("bank_angle_deg",
                          [this]() { return gnc::math::rad2deg(command_.bank_angle_rad); });
        return builder.build();
    }

private:
    static void readSchedule(const gnc::core::ConfigNode& node,
                             std::vector<double>& output) {
        output.clear();
        if (!node.isArray()) {
            return;
        }
        for (size_t i = 0; i < node.size(); ++i) {
            output.push_back(node[i].asDouble());
        }
    }

    double interpolateAngleOfAttackDeg(double altitude_m) const {
        if (schedule_altitude_m_.size() == 1) {
            return schedule_alpha_deg_.front();
        }
        if (altitude_m >= schedule_altitude_m_.front()) {
            return schedule_alpha_deg_.front();
        }
        if (altitude_m <= schedule_altitude_m_.back()) {
            return schedule_alpha_deg_.back();
        }

        for (size_t i = 1; i < schedule_altitude_m_.size(); ++i) {
            if (altitude_m <= schedule_altitude_m_[i - 1] &&
                altitude_m >= schedule_altitude_m_[i]) {
                const double upper_altitude = schedule_altitude_m_[i - 1];
                const double lower_altitude = schedule_altitude_m_[i];
                const double upper_alpha = schedule_alpha_deg_[i - 1];
                const double lower_alpha = schedule_alpha_deg_[i];
                const double ratio =
                    (altitude_m - lower_altitude) /
                    std::max(1.0, upper_altitude - lower_altitude);
                return lower_alpha + ratio * (upper_alpha - lower_alpha);
            }
        }

        return schedule_alpha_deg_.back();
    }

    gnc::plugins::state_3dof::FlightCommand3DOF command_{};
    gnc::plugins::state_3dof::IStateSolver3DOF* state_solver_ = nullptr;
    std::vector<double> schedule_altitude_m_;
    std::vector<double> schedule_alpha_deg_;
    double bank_angle_deg_ = 0.0;
};

} // namespace gnc::vehicle::process
