#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/config_reader.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/forms/local_spherical_6dof/interfaces/i_truth_view.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/vehicle/input/interfaces/i_imu_6dof.hpp"

namespace gnc::vehicle::input {

class IdealImu6Dof final : public gnc::core::ComponentBase,
                           public IImu6Dof,
                           public gnc::interfaces::IObservable {
public:
    IdealImu6Dof() : ComponentBase("IdealImu6Dof") {}

    void configure(const gnc::core::ConfigNode& config,
                   const std::string& config_path) override {
        gnc::core::ConfigReader reader(config, config_path);
        truth_lookup_name_ =
            reader.optionalString("truth_lookup_name", truth_lookup_name_);
        reader.validateNoUnknownKeys();
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(gnc::core::bind(truth_, truth_lookup_name_));
    }

    void initialize() override { update(0.0); }

    void update(double) override {
        const auto& truth = truth_->getLocalSpherical6DoFTruth();
        measurement_.local_acceleration_nue_mps2 =
            truth.local_acceleration_nue_mps2;
        measurement_.angular_rate_body_radps =
            truth.state.angular_rate_body_radps;
        measurement_.angular_acceleration_body_radps2 =
            truth.angular_acceleration_body_radps2;
    }

    const ImuMeasurement6Dof& imuMeasurement6Dof() const override {
        return measurement_;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addVector3("local_acceleration_nue",
                           [this]() -> const gnc::math::Vector3& {
                               return measurement_.local_acceleration_nue_mps2;
                           });
        builder.addVector3("angular_rate_body",
                           [this]() -> const gnc::math::Vector3& {
                               return measurement_.angular_rate_body_radps;
                           });
        builder.addVector3("angular_acceleration_body",
                           [this]() -> const gnc::math::Vector3& {
                               return measurement_.angular_acceleration_body_radps2;
                           });
        return builder.build();
    }

private:
    gnc::forms::local_spherical_6dof::ITruthView* truth_ = nullptr;
    std::string truth_lookup_name_ = "dynamics";
    ImuMeasurement6Dof measurement_{};
};

} // namespace gnc::vehicle::input
