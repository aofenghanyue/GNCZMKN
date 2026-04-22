#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/services/soviet_coord/interfaces/i_coord_service.hpp"

class CoordinateProbe final : public gnc::core::ComponentBase,
                              public gnc::interfaces::IObservable {
public:
    CoordinateProbe() : ComponentBase("CoordinateProbe") {}

    void injectServices(gnc::core::ServiceContext& services) override {
        coord_service_ = services.get<gnc::services::soviet_coord::ICoordService>();
    }

    void configure(const gnc::core::ConfigNode& config) override {
        from_frame_ = config["from_frame"].asString("L");
        to_frame_ = config["to_frame"].asString("K");
    }

    void update(double) override {
        if (!coord_service_) {
            return;
        }
        transformed_axis_ = coord_service_->transform(
            gnc::math::Vector3::UnitX(), from_frame_, to_frame_, getSimTime());
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addVector3("transformed_axis",
                           [this]() -> const gnc::math::Vector3& { return transformed_axis_; });
        return builder.build();
    }

private:
    gnc::services::soviet_coord::ICoordService* coord_service_ = nullptr;
    gnc::math::Vector3 transformed_axis_ = gnc::math::Vector3::UnitX();
    std::string from_frame_ = "L";
    std::string to_frame_ = "K";
};

GNC_REGISTER_COMPONENT_TYPE(
    "example.coordinate_probe",
    CoordinateProbe,
    ::gnc::core::ComponentPackageRole::VehicleProcess,
    ::gnc::core::ExecutionStage::VehicleProcess,
    "local_spherical_3dof",
    gnc::interfaces::IObservable)
