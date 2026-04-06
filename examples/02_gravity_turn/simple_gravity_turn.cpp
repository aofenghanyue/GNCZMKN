#include "gnc/common/logger.hpp"
#include "gnc/components/_builtin_register.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/core/observable_helpers.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/core/simulation_builder.hpp"
#include "gnc/interfaces/gnc/i_guidance_6dof.hpp"
#include "gnc/interfaces/infrastructure/i_observable.hpp"

#include <cmath>

using namespace gnc;
using namespace gnc::core;
using namespace gnc::interfaces;

class GravityTurnGuidance : public ComponentBase,
                            public IGuidance6DOF,
                            public IObservable {
public:
    GravityTurnGuidance() : ComponentBase("GravityTurnGuidance") {
        setExecutionFrequency(50.0);
    }

    void configure(const ConfigNode& config) override {
        thrust_ = config["thrust"].asDouble(50.0);
        pitch_start_time_ = config["pitch_start_time"].asDouble(2.0);
        pitch_rate_ = config["pitch_rate"].asDouble(0.05);
        max_pitch_angle_ = config["max_pitch_angle"].asDouble(1.2);
    }

    void update(double) override {
        double pitch_angle = 0.0;
        if (getSimTime() > pitch_start_time_) {
            pitch_angle = pitch_rate_ * (getSimTime() - pitch_start_time_);
            if (pitch_angle > max_pitch_angle_) {
                pitch_angle = max_pitch_angle_;
            }
        }

        cmd_.acceleration_cmd = {
            thrust_ * std::sin(pitch_angle),
            0.0,
            thrust_ * std::cos(pitch_angle)
        };
        cmd_.timestamp = getSimTime();
    }

    const GuidanceCommand6DOF& getGuidanceCommand() const override {
        return cmd_;
    }

    void setTarget(const Vector3d& target) override {
        target_ = target;
    }

    bool isActive() const override {
        return true;
    }

    std::vector<ObservableField> getObservableFields() const override {
        ObservableFieldBuilder builder;
        builder.addVector3d("acceleration_cmd", [this]() -> const Vector3d& { return cmd_.acceleration_cmd; });
        builder.addScalar("timestamp", [this]() { return cmd_.timestamp; });
        return builder.build();
    }

private:
    GuidanceCommand6DOF cmd_;
    Vector3d target_;
    double thrust_ = 50.0;
    double pitch_start_time_ = 2.0;
    double pitch_rate_ = 0.05;
    double max_pitch_angle_ = 1.2;
};

GNC_REGISTER_COMPONENT(GravityTurnGuidance, IGuidance6DOF)

int main() {
    const char* config_json = R"({
        "simulation": {
            "dt": 0.01,
            "duration": 12.0,
            "integrator": "rk4"
        },
        "outputs": {
            "directory": "user/outputs/{timestamp}",
            "format": "csv",
            "session_name": "gravity_turn",
            "record": {
                "guidance": "all",
                "dynamics": ["position", "velocity", "speed"]
            }
        },
        "components": [
            { "type": "Wgs84Earth", "name": "earth", "config": {} },
            { "type": "GravityTurnGuidance", "name": "guidance", "config": {
                "thrust": 25.0,
                "pitch_start_time": 2.0,
                "pitch_rate": 0.08,
                "max_pitch_angle": 1.0
            }},
            { "type": "SimpleDynamics", "name": "dynamics", "config": {
                "initial_position": [0, 0, 0],
                "initial_velocity": [0, 0, 0]
            }},
            { "type": "IdealImu", "name": "imu", "config": { "frequency_hz": 100 } },
            { "type": "SimpleNavigation", "name": "nav", "config": { "frequency_hz": 50 } }
        ]
    })";

    SimulationBuilder builder;
    if (!builder.loadConfigString(config_json)) {
        LOG_ERROR("Failed to load gravity turn example config. Please check the embedded JSON string.");
        return 1;
    }

    try {
        auto& simulator = builder.build();
        simulator.run();
        return 0;
    } catch (const std::exception& e) {
        LOG_ERROR("Gravity turn example failed: {}. Please review the diagnostics above and fix the reported issue.", e.what());
        return 1;
    }
}
