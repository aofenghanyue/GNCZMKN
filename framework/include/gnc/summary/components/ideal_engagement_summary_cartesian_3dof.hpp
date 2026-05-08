#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/forms/cartesian_3dof/interfaces/i_truth_view.hpp"
#include "gnc/forms/target_point/interfaces/i_truth_view.hpp"
#include "gnc/interfaces/i_summary_observer.hpp"
#include "gnc/vehicle/input/interfaces/i_air_data_3dof.hpp"
#include "gnc/vehicle/output/interfaces/i_constant_mass.hpp"
#include "gnc/vehicle/output/interfaces/i_continuous_mass.hpp"

#include <limits>
#include <ostream>
#include <string>

namespace gnc::summary {

class IdealEngagementSummaryCartesian3Dof final
    : public gnc::core::ComponentBase,
      public gnc::interfaces::ISummaryObserver {
public:
    IdealEngagementSummaryCartesian3Dof()
        : ComponentBase("IdealEngagementSummaryCartesian3Dof") {}

    void configure(const gnc::core::ConfigNode& config) override {
        configure(config, "config");
    }

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        interceptor_truth_lookup_name_ =
            reader.optionalString("interceptor_truth_lookup_name",
                                  interceptor_truth_lookup_name_);
        target_truth_lookup_name_ =
            reader.optionalString("target_truth_lookup_name",
                                  target_truth_lookup_name_);
        mass_lookup_name_ =
            reader.optionalString("mass_lookup_name", mass_lookup_name_);
        air_data_lookup_name_ =
            reader.optionalString("air_data_lookup_name", air_data_lookup_name_);
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(
            gnc::core::bind(interceptor_truth_, interceptor_truth_lookup_name_),
            gnc::core::bind(target_truth_, target_truth_lookup_name_),
            gnc::core::bindIfPresent(constant_mass_, mass_lookup_name_),
            gnc::core::bindIfPresent(continuous_mass_, mass_lookup_name_),
            gnc::core::bindIfPresent(air_data_, air_data_lookup_name_));
    }

    void initialize() override { update(0.0); }

    void update(double) override {
        const auto& interceptor = interceptor_truth_->getCartesian3DoFTruth();
        const auto& target = target_truth_->getTargetPointTruth();
        const double range_m =
            (target.position_ecef_m - interceptor.state.position_m).norm();
        if (range_m < minimum_range_m_) {
            minimum_range_m_ = range_m;
        }

        if (air_data_) {
            const double q = air_data_->airDataMeasurement3Dof().dynamic_pressure_pa;
            if (q > maximum_dynamic_pressure_pa_) {
                maximum_dynamic_pressure_pa_ = q;
            }
        }

        const double acceleration_magnitude =
            interceptor.acceleration_mps2.norm();
        if (acceleration_magnitude > maximum_acceleration_magnitude_mps2_) {
            maximum_acceleration_magnitude_mps2_ = acceleration_magnitude;
        }

        final_interceptor_altitude_m_ = interceptor.state.position_m.z();
        final_interceptor_speed_mps_ = interceptor.state.velocity_mps.norm();
        final_target_position_m_ = target.position_ecef_m;
        final_mass_kg_ = currentMassKg();
    }

    void writeSummary(std::ostream& out) const override {
        out << "Ideal Cartesian 3DOF Engagement Summary\n";
        out << "minimum_range_m: " << minimum_range_m_ << "\n";
        out << "maximum_dynamic_pressure_pa: "
            << maximum_dynamic_pressure_pa_ << "\n";
        out << "maximum_acceleration_magnitude_mps2: "
            << maximum_acceleration_magnitude_mps2_ << "\n";
        out << "final_interceptor_altitude_m: "
            << final_interceptor_altitude_m_ << "\n";
        out << "final_interceptor_speed_mps: "
            << final_interceptor_speed_mps_ << "\n";
        out << "final_target_position_m: "
            << final_target_position_m_.x() << ", "
            << final_target_position_m_.y() << ", "
            << final_target_position_m_.z() << "\n";
        out << "final_mass_kg: " << final_mass_kg_ << "\n";
    }

private:
    double currentMassKg() const {
        if (continuous_mass_) {
            return continuous_mass_->getMassKg();
        }
        if (constant_mass_) {
            return constant_mass_->getMassKg();
        }
        return 0.0;
    }

    gnc::forms::cartesian_3dof::ITruthView* interceptor_truth_ = nullptr;
    gnc::forms::target_point::ITruthView* target_truth_ = nullptr;
    gnc::vehicle::output::IConstantMass* constant_mass_ = nullptr;
    gnc::vehicle::output::IContinuousMass* continuous_mass_ = nullptr;
    gnc::vehicle::input::IAirData3Dof* air_data_ = nullptr;
    std::string interceptor_truth_lookup_name_ = "interceptor.dynamics";
    std::string target_truth_lookup_name_ = "target.truth";
    std::string mass_lookup_name_ = "interceptor.mass";
    std::string air_data_lookup_name_ = "interceptor.air_data";
    double minimum_range_m_ = std::numeric_limits<double>::infinity();
    double maximum_dynamic_pressure_pa_ = 0.0;
    double maximum_acceleration_magnitude_mps2_ = 0.0;
    double final_interceptor_altitude_m_ = 0.0;
    double final_interceptor_speed_mps_ = 0.0;
    gnc::math::Vector3 final_target_position_m_ =
        gnc::math::Vector3::Zero();
    double final_mass_kg_ = 0.0;
};

} // namespace gnc::summary
