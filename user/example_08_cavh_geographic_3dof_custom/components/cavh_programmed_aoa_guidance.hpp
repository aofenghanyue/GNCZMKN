#pragma once

#include "gnc/common/math/eigen_types.hpp"
#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/process/interfaces/i_guidance_3dof.hpp"
#include "gnc/vehicle/process/interfaces/i_navigation_3dof.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace cavh::components {

class ProgrammedAoAGuidance final
    : public gnc::core::ComponentBase,
      public gnc::vehicle::process::IGuidance3Dof,
      public gnc::interfaces::IObservable {
public:
    ProgrammedAoAGuidance() : ComponentBase("CavhProgrammedAoAGuidance") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        navigation_lookup_name_ =
            reader.optionalString("navigation_lookup_name",
                                  navigation_lookup_name_);
        bank_angle_rad_ = gnc::math::deg2rad(
            reader.requiredDouble("bank_angle_deg"));
        auto altitudes = reader.requiredDoubleArray("schedule_altitude_m");
        auto alphas = reader.requiredDoubleArray("schedule_angle_of_attack_deg");
        if (altitudes.empty()) {
            throw std::runtime_error(config_path +
                                     ".schedule_altitude_m must not be empty.");
        }
        if (altitudes.size() != alphas.size()) {
            throw std::runtime_error(
                config_path +
                ".schedule_altitude_m and .schedule_angle_of_attack_deg must have the same length.");
        }

        schedule_.clear();
        schedule_.reserve(altitudes.size());
        for (size_t i = 0; i < altitudes.size(); ++i) {
            schedule_.push_back({altitudes[i], alphas[i]});
        }
        std::sort(schedule_.begin(), schedule_.end(), [](const auto& lhs,
                                                         const auto& rhs) {
            return lhs.first < rhs.first;
        });
        for (size_t i = 1; i < schedule_.size(); ++i) {
            if (schedule_[i - 1].first == schedule_[i].first) {
                throw std::runtime_error(
                    config_path +
                    ".schedule_altitude_m must not contain duplicate nodes.");
            }
        }
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(gnc::core::bind(navigation_, navigation_lookup_name_));
    }

    void initialize() override { update(0.0); }

    void update(double) override {
        altitude_m_ = navigation_->navigationState3Dof().altitude_m;
        command_.angle_of_attack_rad =
            gnc::math::deg2rad(interpolateAngleOfAttackDeg(altitude_m_));
        command_.bank_angle_rad = bank_angle_rad_;
        command_.acceleration_command_nue_mps2 = gnc::math::Vector3::Zero();
    }

    const gnc::vehicle::process::GuidanceCommand3Dof& guidanceCommand3Dof()
        const override {
        return command_;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields()
        const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("altitude_m", [this]() { return altitude_m_; });
        builder.addScalar("angle_of_attack_rad",
                          [this]() { return command_.angle_of_attack_rad; });
        builder.addScalar("angle_of_attack_deg",
                          [this]() {
                              return gnc::math::rad2deg(
                                  command_.angle_of_attack_rad);
                          });
        builder.addScalar("bank_angle_rad",
                          [this]() { return command_.bank_angle_rad; });
        builder.addScalar("bank_angle_deg",
                          [this]() {
                              return gnc::math::rad2deg(command_.bank_angle_rad);
                          });
        return builder.build();
    }

private:
    double interpolateAngleOfAttackDeg(double altitude_m) const {
        if (schedule_.size() == 1) {
            return schedule_.front().second;
        }
        if (altitude_m <= schedule_.front().first) {
            return schedule_.front().second;
        }
        if (altitude_m >= schedule_.back().first) {
            return schedule_.back().second;
        }

        for (size_t i = 1; i < schedule_.size(); ++i) {
            if (altitude_m <= schedule_[i].first) {
                const auto& lower = schedule_[i - 1];
                const auto& upper = schedule_[i];
                const double ratio =
                    (altitude_m - lower.first) / (upper.first - lower.first);
                return lower.second + ratio * (upper.second - lower.second);
            }
        }
        return schedule_.back().second;
    }

    gnc::vehicle::process::INavigation3Dof* navigation_ = nullptr;
    std::string navigation_lookup_name_ = "navigation";
    std::vector<std::pair<double, double>> schedule_;
    gnc::vehicle::process::GuidanceCommand3Dof command_{};
    double bank_angle_rad_ = 0.0;
    double altitude_m_ = 0.0;
};

} // namespace cavh::components

GNC_REGISTER_COMPONENT_TYPE(
    "cavh.process.guidance_3dof.programmed_aoa",
    cavh::components::ProgrammedAoAGuidance,
    ::gnc::core::ComponentPackageRole::VehicleProcess,
    ::gnc::core::ExecutionStage::VehicleProcess,
    "local_spherical_3dof",
    ::gnc::vehicle::process::IGuidance3Dof,
    ::gnc::interfaces::IObservable)
