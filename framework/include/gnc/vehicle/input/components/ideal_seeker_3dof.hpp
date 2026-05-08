#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_truth_view.hpp"
#include "gnc/forms/target_point/interfaces/i_truth_view.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/input/interfaces/i_seeker_3dof.hpp"

namespace gnc::vehicle::input {

class IdealSeeker3Dof final : public gnc::core::ComponentBase,
                              public ISeeker3Dof,
                              public gnc::interfaces::IObservable {
public:
    IdealSeeker3Dof() : ComponentBase("IdealSeeker3Dof") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        truth_lookup_name_ =
            reader.optionalString("truth_lookup_name", truth_lookup_name_);
        target_truth_lookup_name_ =
            reader.requiredString("target_truth_lookup_name");
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(
            gnc::core::bind(truth_, truth_lookup_name_),
            gnc::core::bind(target_truth_, target_truth_lookup_name_));
    }

    void initialize() override { update(0.0); }

    void update(double) override {
        const auto& own = truth_->getLocalSpherical3DoFTruth();
        const auto& target = target_truth_->getTargetPointTruth();
        measurement_.relative_position_ecef_m =
            target.position_ecef_m - own.position_ecef_m;
        measurement_.relative_velocity_ecef_mps =
            target.velocity_ecef_mps - own.velocity_ecef_mps;
        measurement_.range_m = measurement_.relative_position_ecef_m.norm();
        if (measurement_.range_m > 0.0) {
            measurement_.line_of_sight_ecef =
                measurement_.relative_position_ecef_m / measurement_.range_m;
        } else {
            measurement_.line_of_sight_ecef = gnc::math::Vector3::Zero();
        }
        measurement_.closing_speed_mps =
            -measurement_.relative_velocity_ecef_mps.dot(
                measurement_.line_of_sight_ecef);
    }

    const SeekerMeasurement3Dof& seekerMeasurement3Dof() const override {
        return measurement_;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addVector3("relative_position_ecef",
                           [this]() -> const gnc::math::Vector3& {
                               return measurement_.relative_position_ecef_m;
                           });
        builder.addVector3("relative_velocity_ecef",
                           [this]() -> const gnc::math::Vector3& {
                               return measurement_.relative_velocity_ecef_mps;
                           });
        builder.addVector3("line_of_sight_ecef",
                           [this]() -> const gnc::math::Vector3& {
                               return measurement_.line_of_sight_ecef;
                           });
        builder.addScalar("range_m", [this]() { return measurement_.range_m; });
        builder.addScalar("closing_speed_mps",
                          [this]() { return measurement_.closing_speed_mps; });
        return builder.build();
    }

private:
    gnc::forms::local_spherical_3dof::ITruthView* truth_ = nullptr;
    gnc::forms::target_point::ITruthView* target_truth_ = nullptr;
    std::string truth_lookup_name_ = "dynamics";
    std::string target_truth_lookup_name_;
    SeekerMeasurement3Dof measurement_{};
};

} // namespace gnc::vehicle::input
