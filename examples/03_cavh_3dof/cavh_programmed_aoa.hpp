#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/core/observable_helpers.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/interfaces/dynamics/i_dynamics_model.hpp"
#include "gnc/interfaces/gnc/i_guidance_3dof.hpp"
#include "gnc/interfaces/infrastructure/i_observable.hpp"

#include <algorithm>
#include <vector>

class CavhProgrammedAoA : public gnc::core::ComponentBase,
                          public gnc::interfaces::IGuidance3DOF,
                          public gnc::interfaces::IObservable {
public:
    CavhProgrammedAoA() : ComponentBase("CavhProgrammedAoA") {
        setExecutionFrequency(20.0);
    }

    void configure(const gnc::core::ConfigNode& config) override {
        sigma_deg_ = config["sigma_deg"].asDouble(0.0);
        readSchedule(config["schedule_altitude_m"], schedule_altitude_);
        readSchedule(config["schedule_alpha_deg"], schedule_alpha_deg_);

        if (schedule_altitude_.empty() || schedule_alpha_deg_.size() != schedule_altitude_.size()) {
            schedule_altitude_ = {60000.0, 45000.0, 30000.0, 15000.0};
            schedule_alpha_deg_ = {20.0, 15.0, 10.0, 6.0};
        }
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        dynamics_ = registry.getByName<gnc::interfaces::IDynamicsModel>("dynamics");
    }

    void update(double) override {
        const double altitude = dynamics_ ? dynamics_->getStateValue("altitude") : schedule_altitude_.front();
        command_.alpha = deg2rad(interpolateAlpha(altitude));
        command_.sigma = deg2rad(sigma_deg_);
        command_.timestamp = getSimTime();
    }

    const gnc::interfaces::FlightCommand3DOF& getFlightCommand() const override {
        return command_;
    }

    bool isActive() const override {
        return true;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("alpha_deg", [this]() { return rad2deg(command_.alpha); });
        builder.addScalar("sigma_deg", [this]() { return rad2deg(command_.sigma); });
        builder.addScalar("timestamp", [this]() { return command_.timestamp; });
        return builder.build();
    }

private:
    static void readSchedule(const gnc::core::ConfigNode& node,
                             std::vector<double>& values) {
        values.clear();
        if (!node.isArray()) {
            return;
        }
        for (size_t i = 0; i < node.size(); ++i) {
            values.push_back(node[i].asDouble());
        }
    }

    double interpolateAlpha(double altitude) const {
        if (schedule_altitude_.size() == 1) {
            return schedule_alpha_deg_.front();
        }

        if (altitude >= schedule_altitude_.front()) {
            return schedule_alpha_deg_.front();
        }
        if (altitude <= schedule_altitude_.back()) {
            return schedule_alpha_deg_.back();
        }

        for (size_t i = 1; i < schedule_altitude_.size(); ++i) {
            if (altitude <= schedule_altitude_[i - 1] && altitude >= schedule_altitude_[i]) {
                const double h0 = schedule_altitude_[i - 1];
                const double h1 = schedule_altitude_[i];
                const double a0 = schedule_alpha_deg_[i - 1];
                const double a1 = schedule_alpha_deg_[i];
                const double ratio = (altitude - h1) / std::max(1.0, h0 - h1);
                return a1 + ratio * (a0 - a1);
            }
        }
        return schedule_alpha_deg_.back();
    }

    static double deg2rad(double deg) {
        return deg * 3.14159265358979323846 / 180.0;
    }

    static double rad2deg(double rad) {
        return rad * 180.0 / 3.14159265358979323846;
    }

    gnc::interfaces::FlightCommand3DOF command_;
    gnc::interfaces::IDynamicsModel* dynamics_ = nullptr;
    std::vector<double> schedule_altitude_;
    std::vector<double> schedule_alpha_deg_;
    double sigma_deg_ = 0.0;
};

GNC_REGISTER_COMPONENT(CavhProgrammedAoA, gnc::interfaces::IGuidance3DOF)
