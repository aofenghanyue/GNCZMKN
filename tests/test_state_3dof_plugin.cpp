#include "test_support.hpp"

#include "gnc/core/component_registry.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/environment/components/spherical_earth.hpp"
#include "gnc/environment/components/spherical_gravity.hpp"
#include "gnc/environment/components/standard_atmosphere.hpp"
#include "gnc/forms/local_spherical_3dof/components/flight_state_view.hpp"
#include "gnc/forms/local_spherical_3dof/components/point_mass.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_input_provider.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_truth_view.hpp"
#include "gnc/plugins/flight_state_3dof/interfaces/i_flight_state_3dof_soviet_observer.hpp"
#include "gnc/plugins/state_3dof/components/point_mass_cartesian.hpp"
#include "gnc/plugins/state_3dof/interfaces/i_acceleration_provider_3dof.hpp"
#include "gnc/plugins/state_3dof/interfaces/i_flight_command_provider_3dof.hpp"
#include "gnc/interactions/local_spherical_3dof/components/aero_propulsive.hpp"
#include "gnc/vehicle/common/components/cavh_aero_table.hpp"
#include "gnc/vehicle/common/components/cavh_constant_mass.hpp"
#include "gnc/vehicle/common/components/continuous_constant_rate_mass.hpp"

#include <exception>
#include <iostream>
#include <memory>

namespace {

class ConstantGuidance final : public gnc::core::ComponentBase,
                               public gnc::plugins::state_3dof::IFlightCommandProvider3DOF {
public:
    ConstantGuidance() : ComponentBase("ConstantGuidance") {
        command_.angle_of_attack_rad = 15.0 * gnc::math::constants::DEG_TO_RAD;
        command_.bank_angle_rad = 0.0;
    }

    void update(double) override {}

    const gnc::plugins::state_3dof::FlightCommand3DOF& getFlightCommand() const override {
        return command_;
    }

    bool isActive() const override { return true; }

private:
    gnc::plugins::state_3dof::FlightCommand3DOF command_{};
};

gnc::core::ConfigNode makeCartesianConfig() {
    using namespace test_support;
    return object({
        field("initial_position", array({number(0.0), number(0.0), number(1000.0)})),
        field("initial_velocity", array({number(250.0), number(0.0), number(40.0)})),
        field("constant_acceleration", array({number(0.0), number(0.0), number(-9.81)})),
    });
}

gnc::core::ConfigNode makeContinuousMassConfig() {
    using namespace test_support;
    return object({
        field("initial_mass_kg", number(100.0)),
        field("mass_rate_kg_per_s", number(-2.0)),
    });
}

gnc::core::ConfigNode makeCavhMassConfig() {
    using namespace test_support;
    return object({
        field("mass_kg", number(900.0)),
    });
}

gnc::core::ConfigNode makeSphericalConfig() {
    using namespace test_support;
    return object({
        field("launch_azimuth_rad", number(1.5707963267948966)),
        field("initial_state",
              object({
                  field("longitude_rad", number(1.9198621771937625)),
                  field("latitude_rad", number(0.5235987755982988)),
                  field("altitude_m", number(60000.0)),
                  field("speed_mps", number(3200.0)),
                  field("flight_path_angle_rad", number(-0.10)),
                  field("heading_angle_rad", number(-1.5707963267948966)),
              })),
    });
}

} // namespace

int main() {
    try {
        gnc::plugins::state_3dof::PointMassCartesian cartesian;
        cartesian.configure(makeCartesianConfig());

        Eigen::VectorXd cartesian_dx;
        cartesian.computeDerivatives(0.0, cartesian.getState(), cartesian_dx);
        test_support::require(cartesian.getStateLayout().dimension() == 6,
                              "Cartesian 3DOF state dimension must remain 6.");
        test_support::requireNear(cartesian_dx[0], 250.0, 1e-9,
                                  "Cartesian position derivative no longer matches velocity.");
        test_support::requireNear(cartesian_dx[5], -9.81, 1e-9,
                                  "Cartesian acceleration derivative drifted.");
        test_support::requireNear(cartesian.getAltitude(), 1000.0, 1e-9,
                                  "Cartesian altitude accessor returned an unexpected value.");

        gnc::vehicle::common::ContinuousConstantRateMass continuous_mass;
        continuous_mass.configure(makeContinuousMassConfig());
        Eigen::VectorXd mass_dx;
        continuous_mass.computeDerivatives(0.0, continuous_mass.getState(), mass_dx);
        test_support::requireNear(continuous_mass.getMassKg(), 100.0, 1e-9,
                                  "Continuous mass initial value is wrong.");
        test_support::requireNear(continuous_mass.getMassRateKgPerSec(), -2.0, 1e-9,
                                  "Continuous mass rate accessor is wrong.");
        test_support::requireNear(mass_dx[0], -2.0, 1e-9,
                                  "Continuous mass derivative must equal the configured rate.");

        gnc::core::ComponentRegistry registry;
        registry.add<gnc::environment::StandardAtmosphere,
                     gnc::environment::IAtmosphere>(
            "env.atmosphere",
            std::make_unique<gnc::environment::StandardAtmosphere>());
        registry.add<gnc::environment::SphericalGravity,
                     gnc::environment::IGravity>(
            "env.gravity",
            std::make_unique<gnc::environment::SphericalGravity>());
        registry.add<gnc::environment::SphericalEarth,
                     gnc::environment::IEarth>(
            "env.earth",
            std::make_unique<gnc::environment::SphericalEarth>());
        registry.add<ConstantGuidance,
                     gnc::plugins::state_3dof::IFlightCommandProvider3DOF>(
            "missile.guidance",
            std::make_unique<ConstantGuidance>());

        auto mass = std::make_unique<gnc::vehicle::common::CavhConstantMass>();
        auto* mass_ptr = mass.get();
        mass_ptr->configure(makeCavhMassConfig());
        registry.add<gnc::vehicle::common::CavhConstantMass,
                     gnc::vehicle::common::IConstantMass,
                     gnc::interfaces::IObservable>("missile.mass", std::move(mass));

        registry.add<gnc::vehicle::common::CavhAeroTable,
                     gnc::vehicle::common::IAeroModel,
                     gnc::interfaces::IObservable>(
            "missile.aero",
            std::make_unique<gnc::vehicle::common::CavhAeroTable>());

        auto interaction =
            std::make_unique<gnc::interactions::local_spherical_3dof::AeroPropulsive>();
        auto* interaction_ptr = interaction.get();
        registry.add<gnc::interactions::local_spherical_3dof::AeroPropulsive,
                     gnc::forms::local_spherical_3dof::IInputProvider,
                     gnc::plugins::state_3dof::IAccelerationProvider3DOF>(
            "missile.interaction",
            std::move(interaction));

        auto dynamics = std::make_unique<gnc::forms::local_spherical_3dof::PointMass>();
        auto* dynamics_ptr = dynamics.get();
        dynamics_ptr->configure(makeSphericalConfig());
        registry.add<gnc::forms::local_spherical_3dof::PointMass,
                     gnc::interfaces::IContinuousSystem,
                     gnc::plugins::state_3dof::IStateSolver3DOF,
                     gnc::plugins::state_3dof::ISovietSphericalState3DOF,
                     gnc::plugins::state_3dof::IVelocityDirectionProvider,
                     gnc::forms::local_spherical_3dof::ITruthView,
                     gnc::interfaces::IObservable>("missile.dynamics", std::move(dynamics));

        auto observer =
            std::make_unique<gnc::forms::local_spherical_3dof::FlightStateView>();
        auto* observer_ptr = observer.get();
        registry.add<gnc::forms::local_spherical_3dof::FlightStateView,
                     gnc::plugins::flight_state_3dof::IFlightState3DOFSovietObserver,
                     gnc::interfaces::IObservable>(
            "missile.flight_state",
            std::move(observer));

        gnc::core::ScopedRegistry interaction_scoped("missile", registry, "missile.interaction");
        interaction_ptr->injectDependencies(interaction_scoped);

        gnc::core::ScopedRegistry dynamics_scoped("missile", registry, "missile.dynamics");
        dynamics_ptr->injectDependencies(dynamics_scoped);

        gnc::core::ScopedRegistry observer_scoped("missile", registry, "missile.flight_state");
        observer_ptr->injectDependencies(observer_scoped);
        dynamics_ptr->initialize();
        observer_ptr->initialize();

        const auto spherical_state = dynamics_ptr->getSovietSphericalState();
        const auto interaction_acceleration =
            interaction_ptr->computeLocalAccelerationNue(spherical_state, 0.0);

        Eigen::VectorXd spherical_dx;
        dynamics_ptr->computeDerivatives(0.0, dynamics_ptr->getState(), spherical_dx);
        test_support::require(dynamics_ptr->getStateLayout().dimension() == 6,
                              "Soviet spherical 3DOF state dimension must remain 6.");
        test_support::require(spherical_dx[2] < 0.0,
                              "Descending initial condition should reduce altitude.");
        test_support::require(dynamics_ptr->getVelocityInLaunchFrame().x() > 0.0,
                              "Launch-frame velocity should point downrange.");
        test_support::require(dynamics_ptr->getPosition().norm() > 6.3e6,
                              "ECEF position magnitude is inconsistent with altitude.");
        test_support::require(interaction_acceleration.norm() > 1.0,
                              "Interaction acceleration should include gravity and aerodynamic terms.");

        observer_ptr->update(0.0);
        const auto& flight_state = observer_ptr->getFlightState3DOFSoviet();
        test_support::requireNear(flight_state.angle_of_attack_rad,
                                  15.0 * gnc::math::constants::DEG_TO_RAD,
                                  1e-9,
                                  "Observer lost the commanded angle of attack.");
        test_support::require(flight_state.mach_number > 5.0,
                              "Observer Mach number is inconsistent with the initial state.");
        test_support::require(flight_state.dynamic_pressure_pa > 0.0,
                              "Observer dynamic pressure must be positive.");
        test_support::requireNear(flight_state.local_velocity_nue_mps.norm(),
                                  spherical_state.speed_mps,
                                  1e-6,
                                  "Observer local velocity norm drifted from speed.");
        test_support::requireVectorNear(
            flight_state.local_acceleration_nue_mps2,
            interaction_acceleration,
            1e-6,
            "Flight-state view local acceleration disagrees with the interaction output.");
        test_support::requireNear(mass_ptr->getMassKg(),
                                  900.0,
                                  1e-9,
                                  "CAV-H constant mass no longer returns its configured value.");

        std::cout << "state_3dof plugin checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
