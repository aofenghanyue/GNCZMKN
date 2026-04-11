#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/interfaces/state/i_position_provider.hpp"

namespace gnc::user {

class FixedGeodeticProvider : public gnc::core::ComponentBase,
                              public gnc::interfaces::IPositionProvider {
public:
    FixedGeodeticProvider() : ComponentBase("FixedGeodeticProvider") {}

    void configure(const gnc::core::ConfigNode& config) override {
        latitude_rad_ = config["latitude_rad"].asDouble(0.0);
        longitude_rad_ = config["longitude_rad"].asDouble(0.0);
        altitude_m_ = config["altitude_m"].asDouble(0.0);
    }

    void update(double) override {}

    gnc::Vector3d getPosition() const override {
        return {latitude_rad_, longitude_rad_, altitude_m_};
    }

private:
    double latitude_rad_ = 0.0;
    double longitude_rad_ = 0.0;
    double altitude_m_ = 0.0;
};

GNC_REGISTER_COMPONENT(FixedGeodeticProvider, gnc::interfaces::IPositionProvider)

} // namespace gnc::user
