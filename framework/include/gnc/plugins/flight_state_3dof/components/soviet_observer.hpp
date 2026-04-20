#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_continuous_system.hpp"
#include "gnc/interfaces/i_observable.hpp"
#include "gnc/plugins/environment/interfaces/i_atmosphere.hpp"
#include "gnc/plugins/environment/interfaces/i_earth.hpp"
#include "gnc/plugins/flight_state_3dof/interfaces/i_flight_state_3dof_soviet_observer.hpp"
#include "gnc/plugins/state_3dof/interfaces/i_flight_command_provider_3dof.hpp"
#include "gnc/plugins/state_3dof/interfaces/i_soviet_spherical_state_3dof.hpp"
#include "gnc/plugins/state_3dof/internal/soviet_spherical_3dof_math.hpp"

#include <algorithm>
#include <stdexcept>

namespace gnc::plugins::flight_state_3dof {

class SovietObserver final : public gnc::core::ComponentBase,
                             public IFlightState3DOFSovietObserver,
                             public gnc::interfaces::IObservable {
public:
    SovietObserver() : ComponentBase("FlightState3DOFSovietObserver") {}

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        registry.bindAll(
            gnc::core::bind(soviet_state_, "dynamics"),
            gnc::core::bind(continuous_system_, "dynamics"),
            gnc::core::bind(earth_, "env.earth"),
            gnc::core::bind(atmosphere_, "env.atmosphere"),
            gnc::core::bindIfPresent(command_provider_, "guidance"));
    }

    void initialize() override { refreshSample(getSimTime()); }

    void update(double dt) override { refreshSample(getSimTime() + dt); }

    const FlightState3DOFSovietSample& getFlightState3DOFSoviet() const override {
        return sample_;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("longitude_rad", [this]() { return sample_.longitude_rad; });
        builder.addScalar("latitude_rad", [this]() { return sample_.latitude_rad; });
        builder.addScalar("altitude_m", [this]() { return sample_.altitude_m; });
        builder.addScalar("speed_mps", [this]() { return sample_.speed_mps; });
        builder.addScalar("flight_path_angle_rad",
                          [this]() { return sample_.flight_path_angle_rad; });
        builder.addScalar("heading_angle_rad",
                          [this]() { return sample_.heading_angle_rad; });
        builder.addVector3("local_velocity_nue", [this]() -> const gnc::math::Vector3& {
            return sample_.local_velocity_nue_mps;
        });
        builder.addVector3("local_acceleration_nue",
                           [this]() -> const gnc::math::Vector3& {
                               return sample_.local_acceleration_nue_mps2;
                           });
        builder.addScalar("angle_of_attack_rad",
                          [this]() { return sample_.angle_of_attack_rad; });
        builder.addScalar("bank_angle_rad", [this]() { return sample_.bank_angle_rad; });
        builder.addScalar("dynamic_pressure_pa",
                          [this]() { return sample_.dynamic_pressure_pa; });
        builder.addScalar("density_kg_per_m3",
                          [this]() { return sample_.density_kg_per_m3; });
        builder.addScalar("pressure_pa", [this]() { return sample_.pressure_pa; });
        builder.addScalar("temperature_k", [this]() { return sample_.temperature_k; });
        builder.addScalar("speed_of_sound_mps",
                          [this]() { return sample_.speed_of_sound_mps; });
        builder.addScalar("mach_number", [this]() { return sample_.mach_number; });
        return builder.build();
    }

private:
    static double readDerivative(const gnc::interfaces::IContinuousSystem& system,
                                 const Eigen::VectorXd& derivative,
                                 const std::string& name) {
        const int index = system.getStateLayout().indexOf(name);
        if (index < 0 || index >= derivative.size()) {
            throw std::runtime_error(
                "flight_state_3dof.soviet_observer could not find state derivative '" +
                name + "'.");
        }
        return derivative[index];
    }

    void refreshSample(double sample_time) {
        const auto state = soviet_state_->getSovietSphericalState();
        const auto atmosphere_sample = atmosphere_->sample(state.altitude_m);
        const auto command = currentCommand();

        Eigen::VectorXd derivative;
        continuous_system_->computeDerivatives(sample_time,
                                               continuous_system_->getState(),
                                               derivative);

        gnc::plugins::state_3dof::internal::SovietSphericalStateDerivative3DOF
            state_derivative;
        state_derivative.longitude_rate_rad_per_s =
            readDerivative(*continuous_system_, derivative, "longitude_rad");
        state_derivative.latitude_rate_rad_per_s =
            readDerivative(*continuous_system_, derivative, "latitude_rad");
        state_derivative.altitude_rate_m_per_s =
            readDerivative(*continuous_system_, derivative, "altitude_m");
        state_derivative.speed_rate_m_per_s2 =
            readDerivative(*continuous_system_, derivative, "speed_mps");
        state_derivative.flight_path_angle_rate_rad_per_s =
            readDerivative(*continuous_system_, derivative, "flight_path_angle_rad");
        state_derivative.heading_angle_rate_rad_per_s =
            readDerivative(*continuous_system_, derivative, "heading_angle_rad");

        sample_.longitude_rad = state.longitude_rad;
        sample_.latitude_rad = state.latitude_rad;
        sample_.altitude_m = state.altitude_m;
        sample_.speed_mps = state.speed_mps;
        sample_.flight_path_angle_rad = state.flight_path_angle_rad;
        sample_.heading_angle_rad = state.heading_angle_rad;
        sample_.local_velocity_nue_mps =
            gnc::plugins::state_3dof::internal::localVelocityNue(state);
        sample_.local_acceleration_nue_mps2 =
            gnc::plugins::state_3dof::internal::reconstructLocalAccelerationNue(
                state,
                state_derivative,
                earth_->getEquatorialRadius(),
                earth_->getRotationRate());
        sample_.angle_of_attack_rad = command.angle_of_attack_rad;
        sample_.bank_angle_rad = command.bank_angle_rad;
        sample_.density_kg_per_m3 = atmosphere_sample.density_kg_per_m3;
        sample_.pressure_pa = atmosphere_sample.pressure_pa;
        sample_.temperature_k = atmosphere_sample.temperature_k;
        sample_.speed_of_sound_mps = atmosphere_sample.speed_of_sound_mps;
        sample_.dynamic_pressure_pa =
            0.5 * sample_.density_kg_per_m3 * sample_.speed_mps * sample_.speed_mps;
        sample_.mach_number =
            sample_.speed_mps / std::max(1.0, sample_.speed_of_sound_mps);
    }

    gnc::plugins::state_3dof::FlightCommand3DOF currentCommand() const {
        if (command_provider_ && command_provider_->isActive()) {
            return command_provider_->getFlightCommand();
        }
        return {};
    }

    gnc::plugins::state_3dof::ISovietSphericalState3DOF* soviet_state_ = nullptr;
    gnc::interfaces::IContinuousSystem* continuous_system_ = nullptr;
    gnc::plugins::environment::IEarth* earth_ = nullptr;
    gnc::plugins::environment::IAtmosphere* atmosphere_ = nullptr;
    gnc::plugins::state_3dof::IFlightCommandProvider3DOF* command_provider_ = nullptr;
    FlightState3DOFSovietSample sample_{};
};

} // namespace gnc::plugins::flight_state_3dof
