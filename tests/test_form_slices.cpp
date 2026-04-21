#include "test_support.hpp"

#include "gnc/core/component_registry.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/environment/components/spherical_earth.hpp"
#include "gnc/environment/components/spherical_gravity.hpp"
#include "gnc/environment/components/standard_atmosphere.hpp"
#include "gnc/forms/cartesian_3dof/components/point_mass.hpp"
#include "gnc/forms/cartesian_3dof/interfaces/i_input_provider.hpp"
#include "gnc/forms/cartesian_3dof/interfaces/i_truth_view.hpp"
#include "gnc/forms/local_spherical_3dof/components/flight_state_view.hpp"
#include "gnc/forms/local_spherical_3dof/components/point_mass.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_flight_state_view.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_input_provider.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_truth_view.hpp"
#include "gnc/interactions/cartesian_3dof/components/direct_accel.hpp"
#include "gnc/interactions/local_spherical_3dof/components/aero_propulsive.hpp"
#include "gnc/services/soviet_coord/interfaces/i_velocity_direction_provider.hpp"
#include "gnc/vehicle/common/components/constant_mass.hpp"
#include "gnc/vehicle/common/components/continuous_constant_rate_mass.hpp"
#include "gnc/vehicle/common/components/table2d_aero.hpp"
#include "gnc/vehicle/process/interfaces/i_aero_guidance_provider.hpp"

#include <exception>
#include <iostream>
#include <memory>

namespace {

class ConstantGuidance final : public gnc::core::ComponentBase,
                               public gnc::vehicle::process::IAeroGuidanceProvider {
public:
    ConstantGuidance() : ComponentBase("ConstantGuidance") {
        command_.angle_of_attack_rad = 15.0 * gnc::math::constants::DEG_TO_RAD;
        command_.bank_angle_rad = 0.0;
    }

    void update(double) override {}

    const gnc::vehicle::process::AeroGuidanceCommand& getAeroGuidanceCommand() const override {
        return command_;
    }

    bool isGuidanceActive() const override { return true; }

private:
    gnc::vehicle::process::AeroGuidanceCommand command_{};
};

gnc::core::ConfigNode makeCartesianConfig() {
    using namespace test_support;
    return object({
        field("initial_position", array({number(0.0), number(0.0), number(1000.0)})),
        field("initial_velocity", array({number(250.0), number(0.0), number(40.0)})),
    });
}

gnc::core::ConfigNode makeCartesianInteractionConfig() {
    using namespace test_support;
    return object({
        field("acceleration_mps2", array({number(0.0), number(0.0), number(-9.81)})),
    });
}

gnc::core::ConfigNode makeContinuousMassConfig() {
    using namespace test_support;
    return object({
        field("initial_mass_kg", number(100.0)),
        field("mass_rate_kg_per_s", number(-2.0)),
    });
}

gnc::core::ConfigNode makeConstantMassConfig() {
    using namespace test_support;
    return object({
        field("mass_kg", number(900.0)),
    });
}

gnc::core::ConfigNode makeTable2dAeroConfig() {
    using namespace test_support;
    return object({
        field("alpha_breaks_rad",
              array({number(0.17453292519943295),
                     number(0.2617993877991494),
                     number(0.3490658503988659)})),
        field("mach_breaks",
              array({number(3.5),
                     number(5.0),
                     number(8.0),
                     number(10.0),
                     number(15.0),
                     number(20.0),
                     number(23.0)})),
        field("lift_coefficients",
              array({array({number(0.4500),
                            number(0.4250),
                            number(0.4000),
                            number(0.3800),
                            number(0.3700),
                            number(0.3600),
                            number(0.3500)}),
                     array({number(0.7400),
                            number(0.7000),
                            number(0.6700),
                            number(0.6300),
                            number(0.6000),
                            number(0.5700),
                            number(0.5570)}),
                     array({number(1.0500),
                            number(1.0000),
                            number(0.9500),
                            number(0.9000),
                            number(0.8500),
                            number(0.8000),
                            number(0.7800)})})),
        field("drag_coefficients",
              array({array({number(0.2045),
                            number(0.1700),
                            number(0.1290),
                            number(0.1090),
                            number(0.1090),
                            number(0.1090),
                            number(0.1090)}),
                     array({number(0.2960),
                            number(0.2630),
                            number(0.2240),
                            number(0.1970),
                            number(0.1950),
                            number(0.1920),
                            number(0.1920)}),
                     array({number(0.4770),
                            number(0.4230),
                            number(0.3540),
                            number(0.3100),
                            number(0.3050),
                            number(0.3000),
                            number(0.3000)})})),
        field("reference_area_m2", number(0.48387)),
        field("reference_length_m", number(3.0)),
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
        gnc::core::ComponentRegistry cartesian_registry;
        auto cartesian_interaction =
            std::make_unique<gnc::interactions::cartesian_3dof::DirectAccel>();
        cartesian_interaction->configure(makeCartesianInteractionConfig());
        cartesian_registry.add<gnc::interactions::cartesian_3dof::DirectAccel,
                               gnc::forms::cartesian_3dof::IInputProvider>(
            "cartesian.interaction",
            std::move(cartesian_interaction));

        auto cartesian_component = std::make_unique<gnc::forms::cartesian_3dof::PointMass>();
        auto* cartesian_ptr = cartesian_component.get();
        cartesian_ptr->configure(makeCartesianConfig());
        cartesian_registry.add<gnc::forms::cartesian_3dof::PointMass,
                               gnc::interfaces::IContinuousSystem,
                               gnc::forms::cartesian_3dof::ITruthView,
                               gnc::interfaces::IObservable>(
            "cartesian.dynamics",
            std::move(cartesian_component));

        gnc::core::ScopedRegistry cartesian_scoped("cartesian",
                                                   cartesian_registry,
                                                   "cartesian.dynamics");
        cartesian_ptr->injectDependencies(cartesian_scoped);
        cartesian_ptr->initialize();

        Eigen::VectorXd cartesian_dx;
        cartesian_ptr->computeDerivatives(0.0, cartesian_ptr->getState(), cartesian_dx);
        test_support::require(cartesian_ptr->getStateLayout().dimension() == 6,
                              "Cartesian 3DOF state dimension must remain 6.");
        test_support::requireNear(cartesian_dx[0], 250.0, 1e-9,
                                  "Cartesian position derivative no longer matches velocity.");
        test_support::requireNear(cartesian_dx[5], -9.81, 1e-9,
                                  "Cartesian acceleration derivative drifted.");
        test_support::requireNear(cartesian_ptr->getAltitude(), 1000.0, 1e-9,
                                  "Cartesian altitude accessor returned an unexpected value.");
        test_support::requireVectorNear(
            cartesian_ptr->getCartesian3DoFTruth().acceleration_mps2,
            gnc::math::Vector3(0.0, 0.0, -9.81),
            1e-9,
            "Cartesian truth view lost the interaction acceleration.");

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
                     gnc::vehicle::process::IAeroGuidanceProvider>(
            "missile.guidance",
            std::make_unique<ConstantGuidance>());

        auto mass = std::make_unique<gnc::vehicle::common::ConstantMass>();
        auto* mass_ptr = mass.get();
        mass_ptr->configure(makeConstantMassConfig());
        registry.add<gnc::vehicle::common::ConstantMass,
                     gnc::vehicle::common::IConstantMass,
                     gnc::interfaces::IObservable>("missile.mass", std::move(mass));

        auto aero = std::make_unique<gnc::vehicle::common::Table2DAero>();
        aero->configure(makeTable2dAeroConfig());
        registry.add<gnc::vehicle::common::Table2DAero,
                     gnc::vehicle::common::IAeroModel,
                     gnc::interfaces::IObservable>(
            "missile.aero",
            std::move(aero));

        auto interaction =
            std::make_unique<gnc::interactions::local_spherical_3dof::AeroPropulsive>();
        auto* interaction_ptr = interaction.get();
        registry.add<gnc::interactions::local_spherical_3dof::AeroPropulsive,
                     gnc::forms::local_spherical_3dof::IInputProvider>(
            "missile.interaction",
            std::move(interaction));

        auto dynamics = std::make_unique<gnc::forms::local_spherical_3dof::PointMass>();
        auto* dynamics_ptr = dynamics.get();
        dynamics_ptr->configure(makeSphericalConfig());
        registry.add<gnc::forms::local_spherical_3dof::PointMass,
                     gnc::interfaces::IContinuousSystem,
                     gnc::services::soviet_coord::IVelocityDirectionProvider,
                     gnc::forms::local_spherical_3dof::ITruthView,
                     gnc::interfaces::IObservable>("missile.dynamics", std::move(dynamics));

        auto observer =
            std::make_unique<gnc::forms::local_spherical_3dof::FlightStateView>();
        auto* observer_ptr = observer.get();
        registry.add<gnc::forms::local_spherical_3dof::FlightStateView,
                     gnc::forms::local_spherical_3dof::IFlightStateView,
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

        const auto interaction_acceleration =
            interaction_ptr->computeLocalAccelerationNue(
                dynamics_ptr->getLocalSpherical3DoFTruth());

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
        const auto& flight_state = observer_ptr->getFlightState();
        test_support::requireNear(flight_state.angle_of_attack_rad,
                                  15.0 * gnc::math::constants::DEG_TO_RAD,
                                  1e-9,
                                  "Observer lost the commanded angle of attack.");
        test_support::require(flight_state.mach_number > 5.0,
                              "Observer Mach number is inconsistent with the initial state.");
        test_support::require(flight_state.dynamic_pressure_pa > 0.0,
                              "Observer dynamic pressure must be positive.");
        test_support::requireNear(flight_state.local_velocity_nue_mps.norm(),
                                  dynamics_ptr->getLocalSpherical3DoFTruth().state.speed_mps,
                                  1e-6,
                                  "Observer local velocity norm drifted from speed.");
        test_support::requireVectorNear(
            flight_state.local_acceleration_nue_mps2,
            interaction_acceleration,
            1e-6,
            "Flight-state view local acceleration disagrees with the interaction output.");
        test_support::requireNear(flight_state.local_velocity_nue_mps.norm(),
                                  dynamics_ptr->getLocalSpherical3DoFTruth().state.speed_mps,
                                  1e-6,
                                  "Flight-state view local velocity norm drifted from speed.");
        test_support::requireNear(mass_ptr->getMassKg(),
                                  900.0,
                                  1e-9,
                                  "Constant mass no longer returns its configured value.");

        std::cout << "form slice checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
