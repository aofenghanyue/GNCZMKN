#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/core/observable_helpers.hpp"
#include "gnc/interfaces/infrastructure/i_observable.hpp"
#include "gnc/interfaces/vehicle/i_mass_property.hpp"

class CavhMass : public gnc::core::ComponentBase,
                 public gnc::interfaces::IMassProperty,
                 public gnc::interfaces::IObservable {
public:
    CavhMass() : ComponentBase("CavhMass") {}

    void update(double) override {}

    void configure(const gnc::core::ConfigNode& config) override {
        mass_ = config["mass_kg"].asDouble(900.0);
    }

    double getMass() const override {
        return mass_;
    }

    gnc::Matrix3d getInertiaMatrix() const override {
        return gnc::Matrix3d::Identity();
    }

    gnc::Vector3d getCenterOfMass() const override {
        return gnc::Vector3d::Zero();
    }

    void updateMassProperties(double) override {
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("mass", [this]() { return mass_; });
        return builder.build();
    }

private:
    double mass_ = 900.0;
};

GNC_REGISTER_COMPONENT(CavhMass, gnc::interfaces::IMassProperty)
