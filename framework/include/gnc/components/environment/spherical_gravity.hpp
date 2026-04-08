#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/interfaces/environment/i_gravity_model.hpp"

namespace gnc::components {

class SphericalGravity : public core::ComponentBase,
                         public interfaces::IGravityModel {
public:
    SphericalGravity() : ComponentBase("SphericalGravity") {}

    void update(double) override {}

    gnc::Vector3d getGravity(const gnc::Vector3d& position) const override {
        constexpr double kEarthRadius = 6371000.0;
        const double r = position.norm();
        if (r < 1e-6) {
            return {0.0, 0.0, -sea_level_g_};
        }

        const double g = sea_level_g_ * (kEarthRadius * kEarthRadius) / (r * r);
        return {-g * position.x / r, -g * position.y / r, -g * position.z / r};
    }

    double getSeaLevelGravity() const override {
        return sea_level_g_;
    }

private:
    double sea_level_g_ = 9.80665;
};

GNC_REGISTER_STARTER_COMPONENT(SphericalGravity, interfaces::IGravityModel)

} // namespace gnc::components
