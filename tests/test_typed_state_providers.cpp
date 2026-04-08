#include "gnc/components/navigation/simple_navigation.hpp"
#include "gnc/components/sensors/ideal_imu.hpp"
#include "gnc/components/state/truth_state.hpp"
#include "gnc/core/component_registry.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/interfaces/state/i_angular_velocity_provider.hpp"
#include "gnc/interfaces/state/i_attitude_provider.hpp"
#include "gnc/interfaces/state/i_position_provider.hpp"
#include "gnc/interfaces/state/i_velocity_provider.hpp"

#include <cmath>
#include <iostream>
#include <memory>

namespace {

bool nearlyEqual(double lhs, double rhs) {
    return std::abs(lhs - rhs) < 1e-9;
}

bool sameVector(const gnc::Vector3d& lhs, const gnc::Vector3d& rhs) {
    return nearlyEqual(lhs.x, rhs.x) &&
           nearlyEqual(lhs.y, rhs.y) &&
           nearlyEqual(lhs.z, rhs.z);
}

bool sameQuaternion(const gnc::Quaterniond& lhs, const gnc::Quaterniond& rhs) {
    return nearlyEqual(lhs.w, rhs.w) &&
           nearlyEqual(lhs.x, rhs.x) &&
           nearlyEqual(lhs.y, rhs.y) &&
           nearlyEqual(lhs.z, rhs.z);
}

class FullRigidBodySource : public gnc::core::ComponentBase,
                            public gnc::interfaces::IPositionProvider,
                            public gnc::interfaces::IVelocityProvider,
                            public gnc::interfaces::IAttitudeProvider,
                            public gnc::interfaces::IAngularVelocityProvider {
public:
    FullRigidBodySource(gnc::Vector3d position,
                        gnc::Vector3d velocity,
                        gnc::Quaterniond attitude,
                        gnc::Vector3d angular_velocity)
        : ComponentBase("FullRigidBodySource"),
          position_(position),
          velocity_(velocity),
          attitude_(attitude),
          angular_velocity_(angular_velocity) {}

    void update(double) override {}

    gnc::Vector3d getPosition() const override { return position_; }
    gnc::Vector3d getVelocity() const override { return velocity_; }
    gnc::Quaterniond getAttitude() const override { return attitude_; }
    gnc::Vector3d getAngularVelocity() const override { return angular_velocity_; }

private:
    gnc::Vector3d position_;
    gnc::Vector3d velocity_;
    gnc::Quaterniond attitude_;
    gnc::Vector3d angular_velocity_;
};

class TranslationalSource : public gnc::core::ComponentBase,
                            public gnc::interfaces::IPositionProvider,
                            public gnc::interfaces::IVelocityProvider {
public:
    TranslationalSource(gnc::Vector3d position, gnc::Vector3d velocity)
        : ComponentBase("TranslationalSource"),
          position_(position),
          velocity_(velocity) {}

    void update(double) override {}

    gnc::Vector3d getPosition() const override { return position_; }
    gnc::Vector3d getVelocity() const override { return velocity_; }

private:
    gnc::Vector3d position_;
    gnc::Vector3d velocity_;
};

} // namespace

int main() {
    {
        gnc::core::ComponentRegistry registry;
        auto source = std::make_unique<FullRigidBodySource>(
            gnc::Vector3d{1.0, 2.0, 3.0},
            gnc::Vector3d{4.0, 5.0, 6.0},
            gnc::Quaterniond{0.5, 0.5, 0.5, 0.5},
            gnc::Vector3d{0.1, 0.2, 0.3});
        registry.add<FullRigidBodySource,
                     gnc::interfaces::IPositionProvider,
                     gnc::interfaces::IVelocityProvider,
                     gnc::interfaces::IAttitudeProvider,
                     gnc::interfaces::IAngularVelocityProvider>(
            "vehicle.dynamics", std::move(source));

        gnc::core::ScopedRegistry scoped("vehicle", registry);

        gnc::components::SimpleNavigation nav;
        nav.injectDependencies(scoped);
        nav.initialize();
        nav.update(0.0);

        const auto& nav_state = nav.getNavState();
        if (!nav.isValid() ||
            !sameVector(nav_state.position, gnc::Vector3d{1.0, 2.0, 3.0}) ||
            !sameVector(nav_state.velocity, gnc::Vector3d{4.0, 5.0, 6.0}) ||
            !sameQuaternion(nav_state.attitude, gnc::Quaterniond{0.5, 0.5, 0.5, 0.5}) ||
            !sameVector(nav_state.angular_velocity, gnc::Vector3d{0.1, 0.2, 0.3})) {
            std::cerr << "SimpleNavigation failed to consume typed rigid-body providers\n";
            return 1;
        }

        gnc::components::TruthState truth;
        truth.injectDependencies(scoped);
        truth.update(0.0);

        if (!sameVector(truth.getPosition(), gnc::Vector3d{1.0, 2.0, 3.0}) ||
            !sameVector(truth.getVelocity(), gnc::Vector3d{4.0, 5.0, 6.0}) ||
            !sameQuaternion(truth.getAttitude(), gnc::Quaterniond{0.5, 0.5, 0.5, 0.5}) ||
            !sameVector(truth.getAngularVelocity(), gnc::Vector3d{0.1, 0.2, 0.3})) {
            std::cerr << "TruthState failed to mirror typed rigid-body providers\n";
            return 1;
        }

        gnc::components::IdealImu imu;
        imu.injectDependencies(scoped);
        imu.update(0.0);

        if (!sameVector(imu.getImuData().angular_velocity, gnc::Vector3d{0.1, 0.2, 0.3})) {
            std::cerr << "IdealImu failed to consume the angular velocity provider\n";
            return 1;
        }
    }

    {
        gnc::core::ComponentRegistry registry;
        auto source = std::make_unique<TranslationalSource>(
            gnc::Vector3d{7.0, 8.0, 9.0},
            gnc::Vector3d{10.0, 11.0, 12.0});
        registry.add<TranslationalSource,
                     gnc::interfaces::IPositionProvider,
                     gnc::interfaces::IVelocityProvider>(
            "vehicle.dynamics", std::move(source));

        gnc::core::ScopedRegistry scoped("vehicle", registry);

        gnc::components::SimpleNavigation nav;
        nav.injectDependencies(scoped);
        nav.initialize();
        nav.update(0.0);

        const auto& nav_state = nav.getNavState();
        if (!nav.isValid() ||
            !sameQuaternion(nav_state.attitude, gnc::Quaterniond::Identity()) ||
            !sameVector(nav_state.angular_velocity, gnc::Vector3d::Zero())) {
            std::cerr << "SimpleNavigation fallback for optional providers is broken\n";
            return 1;
        }

        gnc::components::TruthState truth;
        truth.injectDependencies(scoped);
        truth.update(0.0);

        if (!sameQuaternion(truth.getAttitude(), gnc::Quaterniond::Identity()) ||
            !sameVector(truth.getAngularVelocity(), gnc::Vector3d::Zero())) {
            std::cerr << "TruthState fallback for optional providers is broken\n";
            return 1;
        }

        gnc::components::IdealImu imu;
        imu.injectDependencies(scoped);
        imu.update(0.0);

        if (!sameVector(imu.getImuData().angular_velocity, gnc::Vector3d::Zero())) {
            std::cerr << "IdealImu fallback for optional providers is broken\n";
            return 1;
        }
    }

    std::cout << "Typed state providers work\n";
    return 0;
}
