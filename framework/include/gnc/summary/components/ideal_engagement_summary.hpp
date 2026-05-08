#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/forms/local_spherical_6dof/interfaces/i_truth_view.hpp"
#include "gnc/forms/target_point/interfaces/i_truth_view.hpp"
#include "gnc/interfaces/i_summary_observer.hpp"
#include "gnc/vehicle/input/interfaces/i_air_data_6dof.hpp"
#include "gnc/vehicle/output/interfaces/i_mass_properties_6dof.hpp"

#include <limits>
#include <ostream>
#include <string>

namespace gnc::summary {

class IdealEngagementSummary6Dof final
    : public gnc::core::ComponentBase,
      public gnc::interfaces::ISummaryObserver {
public:
    IdealEngagementSummary6Dof()
        : ComponentBase("IdealEngagementSummary6Dof") {}

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
            gnc::core::bindIfPresent(mass_, mass_lookup_name_),
            gnc::core::bindIfPresent(air_data_, air_data_lookup_name_));
    }

    void initialize() override { update(0.0); }

    void update(double) override {
        const auto& interceptor =
            interceptor_truth_->getLocalSpherical6DoFTruth();
        const auto& target = target_truth_->getTargetPointTruth();
        const double range_m =
            (target.position_ecef_m - interceptor.position_ecef_m).norm();
        if (range_m < minimum_range_m_) {
            minimum_range_m_ = range_m;
        }

        if (air_data_) {
            const auto& air = air_data_->airDataMeasurement6Dof();
            const double dynamic_pressure_pa =
                0.5 * air.density_kg_per_m3 * air.speed_mps * air.speed_mps;
            if (dynamic_pressure_pa > maximum_dynamic_pressure_pa_) {
                maximum_dynamic_pressure_pa_ = dynamic_pressure_pa;
            }
        }

        const double acceleration_magnitude =
            interceptor.local_acceleration_nue_mps2.norm();
        if (acceleration_magnitude > maximum_acceleration_magnitude_mps2_) {
            maximum_acceleration_magnitude_mps2_ = acceleration_magnitude;
        }

        final_interceptor_altitude_m_ = interceptor.state.altitude_m;
        final_target_position_ecef_m_ = target.position_ecef_m;
        final_mass_kg_ = mass_ ? mass_->massProperties6Dof().mass_kg : 0.0;
    }

    void writeSummary(std::ostream& out) const override {
        out << "Ideal 6DOF Engagement Summary\n";
        out << "minimum_range_m: " << minimum_range_m_ << "\n";
        out << "maximum_dynamic_pressure_pa: "
            << maximum_dynamic_pressure_pa_ << "\n";
        out << "maximum_acceleration_magnitude_mps2: "
            << maximum_acceleration_magnitude_mps2_ << "\n";
        out << "final_interceptor_altitude_m: "
            << final_interceptor_altitude_m_ << "\n";
        out << "final_target_position_ecef_m: "
            << final_target_position_ecef_m_.x() << ", "
            << final_target_position_ecef_m_.y() << ", "
            << final_target_position_ecef_m_.z() << "\n";
        out << "final_mass_kg: " << final_mass_kg_ << "\n";
    }

private:
    gnc::forms::local_spherical_6dof::ITruthView* interceptor_truth_ = nullptr;
    gnc::forms::target_point::ITruthView* target_truth_ = nullptr;
    gnc::vehicle::output::IMassProperties6Dof* mass_ = nullptr;
    gnc::vehicle::input::IAirData6Dof* air_data_ = nullptr;
    std::string interceptor_truth_lookup_name_ = "interceptor.dynamics";
    std::string target_truth_lookup_name_ = "target.truth";
    std::string mass_lookup_name_ = "interceptor.mass";
    std::string air_data_lookup_name_ = "interceptor.air_data";
    double minimum_range_m_ = std::numeric_limits<double>::infinity();
    double maximum_dynamic_pressure_pa_ = 0.0;
    double maximum_acceleration_magnitude_mps2_ = 0.0;
    double final_interceptor_altitude_m_ = 0.0;
    gnc::math::Vector3 final_target_position_ecef_m_ =
        gnc::math::Vector3::Zero();
    double final_mass_kg_ = 0.0;
};

} // namespace gnc::summary
