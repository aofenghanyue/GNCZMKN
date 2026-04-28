#pragma once

#include "gnc/common/math/eigen_types.hpp"
#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_truth_view.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/process/interfaces/i_aero_guidance_provider.hpp"

#include <algorithm>
#include <string>
#include <vector>

namespace gnc::vehicle::process {

class ProgrammedAoAGuidance final
    : public gnc::core::ComponentBase,
      public IAeroGuidanceProvider,
      public gnc::interfaces::IObservable {
public:
    ProgrammedAoAGuidance() : ComponentBase("ProgrammedAoAGuidance") {
        setExecutionFrequency(20.0);
    }

    void configure(const gnc::core::ConfigNode& config) override {
        configure(config, "config");
    }

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        bank_angle_deg_ = reader.requiredDouble("bank_angle_deg");
        schedule_altitude_m_ = reader.requiredDoubleArray("schedule_altitude_m");
        schedule_alpha_deg_ =
            reader.requiredDoubleArray("schedule_angle_of_attack_deg");

        if (schedule_altitude_m_.empty()) {
            throw std::runtime_error(config_path +
                                     ".schedule_altitude_m must contain at least 1 number.");
        }
        if (schedule_altitude_m_.size() != schedule_alpha_deg_.size()) {
            throw std::runtime_error(
                config_path +
                ".schedule_altitude_m and .schedule_angle_of_attack_deg must have the same length.");
        }
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(gnc::core::bind(truth_view_, "dynamics"));
    }

    void update(double) override {
        const double altitude =
            truth_view_ ? truth_view_->getLocalSpherical3DoFTruth().state.altitude_m
                        : schedule_altitude_m_.front();
        command_.angle_of_attack_rad =
            gnc::math::deg2rad(interpolateAngleOfAttackDeg(altitude));
        command_.bank_angle_rad = gnc::math::deg2rad(bank_angle_deg_);
        command_.timestamp = getSimTime();
    }

    const AeroGuidanceCommand& getAeroGuidanceCommand() const override {
        return command_;
    }

    bool isGuidanceActive() const override { return true; }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("angle_of_attack_deg",
                          [this]() { return gnc::math::rad2deg(command_.angle_of_attack_rad); });
        builder.addScalar("bank_angle_deg",
                          [this]() { return gnc::math::rad2deg(command_.bank_angle_rad); });
        return builder.build();
    }

private:
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

    AeroGuidanceCommand command_{};
    gnc::forms::local_spherical_3dof::ITruthView* truth_view_ = nullptr;
    std::vector<double> schedule_altitude_m_;
    std::vector<double> schedule_alpha_deg_;
    double bank_angle_deg_ = 0.0;
};

} // namespace gnc::vehicle::process
