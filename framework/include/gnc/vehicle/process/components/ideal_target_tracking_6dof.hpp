#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/input/interfaces/i_seeker_6dof.hpp"
#include "gnc/vehicle/process/interfaces/i_target_tracking_6dof.hpp"

namespace gnc::vehicle::process {

class IdealTargetTracking6Dof final : public gnc::core::ComponentBase,
                                      public ITargetTracking6Dof,
                                      public gnc::interfaces::IObservable {
public:
    IdealTargetTracking6Dof()
        : ComponentBase("IdealTargetTracking6Dof") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        seeker_lookup_name_ =
            reader.optionalString("seeker_lookup_name", seeker_lookup_name_);
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(gnc::core::bind(seeker_, seeker_lookup_name_));
    }

    void initialize() override { update(0.0); }

    void update(double) override {
        const auto& seeker = seeker_->seekerMeasurement6Dof();
        track_.relative_position_ecef_m = seeker.relative_position_ecef_m;
        track_.relative_velocity_ecef_mps = seeker.relative_velocity_ecef_mps;
        track_.line_of_sight_ecef = seeker.line_of_sight_ecef;
        track_.range_m = seeker.range_m;
        track_.closing_speed_mps = seeker.closing_speed_mps;
    }

    const TargetTrack6Dof& targetTrack6Dof() const override { return track_; }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addVector3("relative_position_ecef",
                           [this]() -> const gnc::math::Vector3& {
                               return track_.relative_position_ecef_m;
                           });
        builder.addVector3("line_of_sight_ecef",
                           [this]() -> const gnc::math::Vector3& {
                               return track_.line_of_sight_ecef;
                           });
        builder.addScalar("range_m", [this]() { return track_.range_m; });
        builder.addScalar("closing_speed_mps",
                          [this]() { return track_.closing_speed_mps; });
        return builder.build();
    }

private:
    gnc::vehicle::input::ISeeker6Dof* seeker_ = nullptr;
    std::string seeker_lookup_name_ = "seeker";
    TargetTrack6Dof track_{};
};

} // namespace gnc::vehicle::process
