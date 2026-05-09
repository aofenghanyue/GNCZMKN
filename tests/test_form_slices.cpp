#include "test_support.hpp"

#include "gnc/core/component_registry.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/environment/components/spherical_earth.hpp"
#include "gnc/forms/cartesian_3dof/components/point_mass.hpp"
#include "gnc/forms/cartesian_3dof/interfaces/i_input_provider.hpp"
#include "gnc/forms/cartesian_3dof/interfaces/i_truth_view.hpp"
#include "gnc/forms/local_spherical_3dof/components/point_mass.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_input_provider.hpp"
#include "gnc/forms/local_spherical_3dof/interfaces/i_truth_view.hpp"

#include <exception>
#include <iostream>
#include <memory>

namespace {

class CartesianConstantInput final
    : public gnc::core::ComponentBase,
      public gnc::forms::cartesian_3dof::IInputProvider {
public:
    explicit CartesianConstantInput(gnc::math::Vector3 acceleration_mps2)
        : ComponentBase("CartesianConstantInput"),
          acceleration_mps2_(std::move(acceleration_mps2)) {}

    void update(double) override {}

    gnc::forms::cartesian_3dof::Input computeCartesian3DoFInput(
        const gnc::forms::cartesian_3dof::Truth&,
        double) const override {
        gnc::forms::cartesian_3dof::Input input;
        input.acceleration_mps2 = acceleration_mps2_;
        return input;
    }

private:
    gnc::math::Vector3 acceleration_mps2_ = gnc::math::Vector3::Zero();
};

class LocalSphericalConstantInput final
    : public gnc::core::ComponentBase,
      public gnc::forms::local_spherical_3dof::IInputProvider {
public:
    explicit LocalSphericalConstantInput(gnc::math::Vector3 acceleration_nue_mps2)
        : ComponentBase("LocalSphericalConstantInput"),
          acceleration_nue_mps2_(std::move(acceleration_nue_mps2)) {}

    void update(double) override {}

    gnc::forms::local_spherical_3dof::Input computeLocalSpherical3DoFInput(
        const gnc::forms::local_spherical_3dof::Truth&,
        double) const override {
        gnc::forms::local_spherical_3dof::Input input;
        input.local_acceleration_nue_mps2 = acceleration_nue_mps2_;
        return input;
    }

private:
    gnc::math::Vector3 acceleration_nue_mps2_ = gnc::math::Vector3::Zero();
};

gnc::core::ConfigNode makeCartesianConfig() {
    using namespace test_support;
    return object({
        field("initial_position", array({number(0.0), number(0.0), number(1000.0)})),
        field("initial_velocity", array({number(250.0), number(0.0), number(40.0)})),
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
        const gnc::math::Vector3 cartesian_acceleration(0.0, 0.0, -9.81);
        gnc::core::ComponentRegistry cartesian_registry;
        cartesian_registry.add<CartesianConstantInput,
                               gnc::forms::cartesian_3dof::IInputProvider>(
            "cartesian.interaction",
            std::make_unique<CartesianConstantInput>(cartesian_acceleration));

        auto cartesian_component =
            std::make_unique<gnc::forms::cartesian_3dof::PointMass>();
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
            cartesian_acceleration,
            1e-9,
            "Cartesian truth view lost the interaction acceleration.");

        const gnc::math::Vector3 local_acceleration(0.0, -9.80665, 0.0);
        gnc::core::ComponentRegistry spherical_registry;
        spherical_registry.add<gnc::environment::SphericalEarth,
                               gnc::environment::IEarth>(
            "env.earth",
            std::make_unique<gnc::environment::SphericalEarth>());
        spherical_registry.add<LocalSphericalConstantInput,
                               gnc::forms::local_spherical_3dof::IInputProvider>(
            "missile.interaction",
            std::make_unique<LocalSphericalConstantInput>(local_acceleration));

        auto dynamics =
            std::make_unique<gnc::forms::local_spherical_3dof::PointMass>();
        auto* dynamics_ptr = dynamics.get();
        dynamics_ptr->configure(makeSphericalConfig());
        spherical_registry.add<gnc::forms::local_spherical_3dof::PointMass,
                               gnc::interfaces::IContinuousSystem,
                               gnc::forms::local_spherical_3dof::ITruthView,
                               gnc::interfaces::IObservable>(
            "missile.dynamics",
            std::move(dynamics));

        gnc::core::ScopedRegistry dynamics_scoped("missile",
                                                  spherical_registry,
                                                  "missile.dynamics");
        dynamics_ptr->injectDependencies(dynamics_scoped);
        dynamics_ptr->initialize();

        Eigen::VectorXd spherical_dx;
        dynamics_ptr->computeDerivatives(0.0, dynamics_ptr->getState(), spherical_dx);
        test_support::require(dynamics_ptr->getStateLayout().dimension() == 6,
                              "Local-spherical 3DOF state dimension must remain 6.");
        test_support::require(spherical_dx[2] < 0.0,
                              "Descending initial condition should reduce altitude.");
        test_support::require(dynamics_ptr->getVelocityInLaunchFrame().x() > 0.0,
                              "Launch-frame velocity should point downrange.");
        test_support::require(dynamics_ptr->getPosition().norm() > 6.3e6,
                              "ECEF position magnitude is inconsistent with altitude.");
        test_support::requireVectorNear(
            dynamics_ptr->getLocalSpherical3DoFTruth().local_acceleration_nue_mps2,
            local_acceleration,
            1e-9,
            "Local-spherical truth view lost the interaction acceleration.");

        std::cout << "form slice checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
