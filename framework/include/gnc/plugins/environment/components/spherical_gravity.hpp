#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/plugins/environment/interfaces/i_gravity.hpp"

namespace gnc::plugins::environment {

class SphericalGravity final : public gnc::core::ComponentBase,
                               public IGravity {
public:
    SphericalGravity() : ComponentBase("SphericalGravity") {}

    void update(double) override {}

    void configure(const gnc::core::ConfigNode& config) override {
        if (config.isNull()) {
            return;
        }
        reference_radius_m_ =
            config["reference_radius_m"].asDouble(reference_radius_m_);
        sea_level_gravity_mps2_ =
            config["sea_level_gravity_mps2"].asDouble(sea_level_gravity_mps2_);
    }

    double getSeaLevelGravity() const override {
        return sea_level_gravity_mps2_;
    }

    double getGravityMagnitude(double altitude_m) const override {
        const double radius = reference_radius_m_ + altitude_m;
        return sea_level_gravity_mps2_ * reference_radius_m_ * reference_radius_m_ /
               (radius * radius);
    }

    gnc::math::Vector3 getGravityVector(
        const gnc::math::Vector3& position_ecef) const override {
        const double radius = position_ecef.norm();
        if (radius < 1e-9) {
            return gnc::math::Vector3(0.0, 0.0, -sea_level_gravity_mps2_);
        }
        const double magnitude = sea_level_gravity_mps2_ *
                                 reference_radius_m_ * reference_radius_m_ /
                                 (radius * radius);
        return -magnitude * position_ecef.normalized();
    }

private:
    double reference_radius_m_ = 6371000.0;
    double sea_level_gravity_mps2_ = 9.80665;
};

} // namespace gnc::plugins::environment
