#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/forms/local_spherical_6dof/interfaces/i_truth_view.hpp"
#include "gnc/forms/target_point/interfaces/i_truth_view.hpp"
#include "gnc/interfaces/i_termination_evaluator.hpp"

#include <cmath>
#include <string>

namespace gnc::termination {

class EngagementTermination6Dof final
    : public gnc::core::ComponentBase,
      public gnc::interfaces::ITerminationEvaluator {
public:
    EngagementTermination6Dof()
        : ComponentBase("EngagementTermination6Dof") {}

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
        min_range_m_ = reader.optionalDouble("min_range_m", min_range_m_);
        min_altitude_m_ = reader.optionalDouble("min_altitude_m", min_altitude_m_);
        max_time_s_ = reader.optionalDouble("max_time_s", max_time_s_);
        stop_on_non_finite_state_ =
            reader.optionalBool("stop_on_non_finite_state",
                                stop_on_non_finite_state_);
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(
            gnc::core::bind(interceptor_truth_, interceptor_truth_lookup_name_),
            gnc::core::bind(target_truth_, target_truth_lookup_name_));
    }

    void initialize() override { update(0.0); }

    void update(double) override {
        should_terminate_ = false;
        reason_.clear();

        const auto& interceptor =
            interceptor_truth_->getLocalSpherical6DoFTruth();
        const auto& target = target_truth_->getTargetPointTruth();
        const double range_m =
            (target.position_ecef_m - interceptor.position_ecef_m).norm();

        if (stop_on_non_finite_state_ &&
            (!isFinite(interceptor) || !target.position_ecef_m.allFinite() ||
             !target.velocity_ecef_mps.allFinite())) {
            should_terminate_ = true;
            reason_ = "non-finite 6DOF engagement state";
            return;
        }

        if (range_m <= min_range_m_) {
            should_terminate_ = true;
            reason_ = "range limit reached";
            return;
        }

        if (interceptor.state.altitude_m <= min_altitude_m_) {
            should_terminate_ = true;
            reason_ = "altitude limit reached";
            return;
        }

        if (getSimTime() >= max_time_s_) {
            should_terminate_ = true;
            reason_ = "time limit reached";
        }
    }

    bool shouldTerminate() const override { return should_terminate_; }

    std::string reason() const override { return reason_; }

private:
    static bool isFinite(
        const gnc::forms::local_spherical_6dof::Truth& truth) {
        return std::isfinite(truth.state.longitude_rad) &&
               std::isfinite(truth.state.latitude_rad) &&
               std::isfinite(truth.state.altitude_m) &&
               truth.state.velocity_nue_mps.allFinite() &&
               std::isfinite(truth.state.attitude_body_to_nue.w()) &&
               std::isfinite(truth.state.attitude_body_to_nue.x()) &&
               std::isfinite(truth.state.attitude_body_to_nue.y()) &&
               std::isfinite(truth.state.attitude_body_to_nue.z()) &&
               truth.state.angular_rate_body_radps.allFinite() &&
               truth.position_ecef_m.allFinite() &&
               truth.velocity_ecef_mps.allFinite();
    }

    gnc::forms::local_spherical_6dof::ITruthView* interceptor_truth_ = nullptr;
    gnc::forms::target_point::ITruthView* target_truth_ = nullptr;
    std::string interceptor_truth_lookup_name_ = "interceptor.dynamics";
    std::string target_truth_lookup_name_ = "target.truth";
    double min_range_m_ = 5.0;
    double min_altitude_m_ = 0.0;
    double max_time_s_ = 60.0;
    bool stop_on_non_finite_state_ = true;
    bool should_terminate_ = false;
    std::string reason_;
};

} // namespace gnc::termination
