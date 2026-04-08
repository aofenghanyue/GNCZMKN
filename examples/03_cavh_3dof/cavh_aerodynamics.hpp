#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/core/observable_helpers.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/interfaces/gnc/i_guidance_3dof.hpp"
#include "gnc/interfaces/infrastructure/i_observable.hpp"
#include "gnc/interfaces/state/i_velocity_provider.hpp"
#include "gnc/interfaces/vehicle/i_aero_coefficients.hpp"

#include <cmath>

class CavhAerodynamics : public gnc::core::ComponentBase,
                         public gnc::interfaces::IAeroCoefficients,
                         public gnc::interfaces::IObservable {
public:
    CavhAerodynamics() : ComponentBase("CavhAerodynamics") {}

    void update(double) override {
        const double alpha = currentAlpha();
        const double speed = currentSpeed();
        const double mach = speed / 340.0;
        const auto coeffs = computeCoefficients(alpha, 0.0, mach);

        snapDebug("alpha_rad", alpha);
        snapDebug("speed", speed);
        snapDebug("mach", mach);
        snapDebug("CL", coeffs.CL);
        snapDebug("CD", coeffs.CD);
    }

    void configure(const gnc::core::ConfigNode& config) override {
        cl0_ = config["cl0"].asDouble(0.0);
        cl_alpha_ = config["cl_alpha_per_rad"].asDouble(1.6);
        cd0_ = config["cd0"].asDouble(0.08);
        cd2_ = config["cd2"].asDouble(1.25);
        reference_area_ = config["reference_area"].asDouble(0.48);
        reference_length_ = config["reference_length"].asDouble(2.5);
    }

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(
            gnc::core::bind(guidance_, "guidance"),
            gnc::core::bind(velocity_provider_, "dynamics"));
    }

    gnc::interfaces::AeroCoefficients computeCoefficients(double alpha,
                                                          double,
                                                          double mach) const override {
        const double cl = cl0_ + cl_alpha_ * alpha;
        const double cd = cd0_ + cd2_ * alpha * alpha + 0.015 * std::max(0.0, mach - 5.0);
        return {cl, cd, 0.0, 0.0, 0.0, 0.0};
    }

    double getReferenceArea() const override {
        return reference_area_;
    }

    double getReferenceLength() const override {
        return reference_length_;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("CL", [this]() { return currentCoefficients().CL; });
        builder.addScalar("CD", [this]() { return currentCoefficients().CD; });
        builder.addScalar("lift_to_drag", [this]() {
            const auto coeffs = currentCoefficients();
            return std::abs(coeffs.CD) > 1e-9 ? coeffs.CL / coeffs.CD : 0.0;
        });
        return builder.build();
    }

private:
    double currentAlpha() const {
        return guidance_ ? guidance_->getFlightCommand().alpha : 0.0;
    }

    double currentSpeed() const {
        return velocity_provider_ ? velocity_provider_->getVelocity().norm() : 0.0;
    }

    gnc::interfaces::AeroCoefficients currentCoefficients() const {
        const double alpha = currentAlpha();
        const double speed = currentSpeed();
        const double mach = speed / 340.0;
        return computeCoefficients(alpha, 0.0, mach);
    }

    double cl0_ = 0.0;
    double cl_alpha_ = 1.6;
    double cd0_ = 0.08;
    double cd2_ = 1.25;
    double reference_area_ = 0.48;
    double reference_length_ = 2.5;
    gnc::interfaces::IGuidance3DOF* guidance_ = nullptr;
    gnc::interfaces::IVelocityProvider* velocity_provider_ = nullptr;
};

GNC_REGISTER_COMPONENT(CavhAerodynamics, gnc::interfaces::IAeroCoefficients)
