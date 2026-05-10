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
            reader.optionalString(kNavigationLookupNameKey,
                                  navigation_lookup_name_);
        bank_angle_rad_ = gnc::math::deg2rad(
            reader.requiredDouble(kBankAngleDegKey));

        const auto mach_numbers =
            reader.requiredDoubleArray(kScheduleMachNumberKey);
        const auto angle_of_attack_deg =
            reader.requiredDoubleArray(kScheduleAngleOfAttackDegKey);
        angle_of_attack_schedule_ =
            makeAngleOfAttackSchedule(mach_numbers,
                                      angle_of_attack_deg,
                                      config_path);
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(gnc::core::bind(navigation_, navigation_lookup_name_));
    }

    void initialize() override { update(0.0); }

    void update(double) override {
        mach_number_ = navigation_->navigationState3Dof().mach_number;
        command_.angle_of_attack_rad =
            gnc::math::deg2rad(interpolateAngleOfAttackDeg(mach_number_));
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
        builder.addScalar("mach_number", [this]() { return mach_number_; });
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
    struct AngleOfAttackScheduleNode {
        double mach_number = 0.0;
        double angle_of_attack_deg = 0.0;
    };

    static constexpr const char* kNavigationLookupNameKey =
        "navigation_lookup_name";
    static constexpr const char* kBankAngleDegKey = "bank_angle_deg";
    static constexpr const char* kScheduleMachNumberKey =
        "schedule_mach_number";
    static constexpr const char* kScheduleAngleOfAttackDegKey =
        "schedule_angle_of_attack_deg";

    static std::string configFieldPath(const std::string& config_path,
                                       const char* key) {
        if (config_path.empty()) {
            return key;
        }
        return config_path + "." + key;
    }

    static std::vector<AngleOfAttackScheduleNode> makeAngleOfAttackSchedule(
        const std::vector<double>& mach_numbers,
        const std::vector<double>& angle_of_attack_deg,
        const std::string& config_path) {
        if (mach_numbers.empty()) {
            throw std::runtime_error(
                configFieldPath(config_path, kScheduleMachNumberKey) +
                " must not be empty.");
        }
        if (mach_numbers.size() != angle_of_attack_deg.size()) {
            throw std::runtime_error(
                configFieldPath(config_path, kScheduleMachNumberKey) +
                " and " +
                configFieldPath(config_path, kScheduleAngleOfAttackDegKey) +
                " must have the same length.");
        }

        std::vector<AngleOfAttackScheduleNode> schedule;
        schedule.reserve(mach_numbers.size());
        for (size_t i = 0; i < mach_numbers.size(); ++i) {
            schedule.push_back({mach_numbers[i], angle_of_attack_deg[i]});
        }

        std::sort(schedule.begin(),
                  schedule.end(),
                  [](const auto& lhs, const auto& rhs) {
                      return lhs.mach_number < rhs.mach_number;
                  });
        for (size_t i = 1; i < schedule.size(); ++i) {
            if (schedule[i - 1].mach_number == schedule[i].mach_number) {
                throw std::runtime_error(
                    configFieldPath(config_path, kScheduleMachNumberKey) +
                    " must not contain duplicate nodes.");
            }
        }
        return schedule;
    }

    double interpolateAngleOfAttackDeg(double mach_number) const {
        if (angle_of_attack_schedule_.size() == 1) {
            return angle_of_attack_schedule_.front().angle_of_attack_deg;
        }
        if (mach_number <= angle_of_attack_schedule_.front().mach_number) {
            return angle_of_attack_schedule_.front().angle_of_attack_deg;
        }
        if (mach_number >= angle_of_attack_schedule_.back().mach_number) {
            return angle_of_attack_schedule_.back().angle_of_attack_deg;
        }

        for (size_t i = 1; i < angle_of_attack_schedule_.size(); ++i) {
            if (mach_number <= angle_of_attack_schedule_[i].mach_number) {
                const auto& lower = angle_of_attack_schedule_[i - 1];
                const auto& upper = angle_of_attack_schedule_[i];
                const double ratio =
                    (mach_number - lower.mach_number) /
                    (upper.mach_number - lower.mach_number);
                return lower.angle_of_attack_deg +
                       ratio * (upper.angle_of_attack_deg -
                                lower.angle_of_attack_deg);
            }
        }
        return angle_of_attack_schedule_.back().angle_of_attack_deg;
    }

    gnc::vehicle::process::INavigation3Dof* navigation_ = nullptr;
    std::string navigation_lookup_name_ = "navigation";
    std::vector<AngleOfAttackScheduleNode> angle_of_attack_schedule_;
    gnc::vehicle::process::GuidanceCommand3Dof command_{};
    double bank_angle_rad_ = 0.0;
    double mach_number_ = 0.0;
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
