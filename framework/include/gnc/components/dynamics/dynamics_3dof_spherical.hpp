#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/core/observable_helpers.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/core/state_layout.hpp"
#include "gnc/interfaces/dynamics/i_dynamics_model.hpp"
#include "gnc/interfaces/environment/i_atmosphere_model.hpp"
#include "gnc/interfaces/environment/i_gravity_model.hpp"
#include "gnc/interfaces/gnc/i_guidance_3dof.hpp"
#include "gnc/interfaces/infrastructure/i_observable.hpp"
#include "gnc/interfaces/state/i_altitude_provider.hpp"
#include "gnc/interfaces/state/i_position_provider.hpp"
#include "gnc/interfaces/state/i_velocity_provider.hpp"
#include "gnc/interfaces/vehicle/i_aero_coefficients.hpp"
#include "gnc/interfaces/vehicle/i_mass_property.hpp"

#include <algorithm>
#include <cmath>

namespace gnc::components {

class Dynamics3DOF_SphericalEarth : public core::ComponentBase,
                                    public interfaces::IDynamicsModel,
                                    public interfaces::IAltitudeProvider,
                                    public interfaces::IPositionProvider,
                                    public interfaces::IVelocityProvider,
                                    public interfaces::IObservable {
public:
    Dynamics3DOF_SphericalEarth() : ComponentBase("Dynamics3DOF_SphericalEarth") {
        idx_lon_ = layout_.addVariable("longitude");
        idx_lat_ = layout_.addVariable("latitude");
        idx_h_ = layout_.addVariable("altitude");
        idx_v_ = layout_.addVariable("velocity");
        idx_gamma_ = layout_.addVariable("flight_path_angle");
        idx_psi_ = layout_.addVariable("heading_angle");
        state_ = Eigen::VectorXd::Zero(layout_.dimension());
        initial_state_ = state_;
    }

    void configure(const core::ConfigNode& config) override {
        const auto& initial = config["initial_state"];
        initial_state_[idx_lon_] = deg2rad(initial["longitude"].asDouble(0.0));
        initial_state_[idx_lat_] = deg2rad(initial["latitude"].asDouble(0.0));
        initial_state_[idx_h_] = initial["altitude"].asDouble(30000.0);
        initial_state_[idx_v_] = initial["velocity"].asDouble(3500.0);
        initial_state_[idx_gamma_] = deg2rad(initial["flight_path_angle"].asDouble(-5.0));
        initial_state_[idx_psi_] = deg2rad(initial["heading_angle"].asDouble(90.0));
        state_ = initial_state_;
    }

    void injectDependencies(core::ScopedRegistry& registry) override {
        registry.bindAll(
            core::bind(atmosphere_, "atmosphere"),
            core::bind(gravity_, "gravity"),
            core::bind(aero_, "aero"),
            core::bind(mass_, "mass"),
            core::bind(guidance_, "guidance"));
    }

    const core::StateLayout& getStateLayout() const override {
        return layout_;
    }

    void computeDerivatives(double,
                            const Eigen::VectorXd& x,
                            Eigen::VectorXd& dxdt) const override {
        dxdt = Eigen::VectorXd::Zero(layout_.dimension());

        const double lat = x[idx_lat_];
        const double altitude = x[idx_h_];
        const double velocity = std::max(1.0, x[idx_v_]);
        const double gamma = x[idx_gamma_];
        const double psi = x[idx_psi_];
        const double alpha = guidance_ ? guidance_->getFlightCommand().alpha : 0.0;
        const double sigma = guidance_ ? guidance_->getFlightCommand().sigma : 0.0;
        const double mass = mass_ ? std::max(1.0, mass_->getMass()) : 1.0;
        const double g = gravity_ ? gravity_->getGravityAtAltitude(altitude) : 9.80665;
        const double rho = atmosphere_ ? atmosphere_->getDensity(altitude) : 0.0;
        const double a = atmosphere_ ? atmosphere_->getSpeedOfSound(altitude) : 340.0;
        const double mach = velocity / std::max(1.0, a);
        const auto coeffs = aero_ ? aero_->computeCoefficients(alpha, 0.0, mach)
                                  : interfaces::AeroCoefficients{};
        const double q = 0.5 * rho * velocity * velocity;
        const double s_ref = aero_ ? aero_->getReferenceArea() : 1.0;
        const double lift = q * s_ref * coeffs.CL;
        const double drag = q * s_ref * coeffs.CD;
        const double r = earth_radius_ + altitude;
        const double cos_lat = std::max(1e-6, std::cos(lat));
        const double cos_gamma = std::max(1e-6, std::cos(gamma));

        dxdt[idx_lon_] = velocity * cos_gamma * std::sin(psi) / (r * cos_lat);
        dxdt[idx_lat_] = velocity * cos_gamma * std::cos(psi) / r;
        dxdt[idx_h_] = velocity * std::sin(gamma);
        dxdt[idx_v_] = -drag / mass - g * std::sin(gamma);
        dxdt[idx_gamma_] = (lift * std::cos(sigma)) / (mass * velocity)
                         + (velocity / r - g / velocity) * cos_gamma;
        dxdt[idx_psi_] = (lift * std::sin(sigma)) / (mass * velocity * cos_gamma)
                       + velocity * cos_gamma * std::sin(psi) * std::tan(lat) / r;
    }

    const Eigen::VectorXd& getState() const override {
        return state_;
    }

    void setState(const Eigen::VectorXd& x) override {
        state_ = x;
    }

    Eigen::VectorXd getInitialState() const override {
        return initial_state_;
    }

    gnc::Vector3d getPosition() const override {
        const double lon = state_[idx_lon_];
        const double lat = state_[idx_lat_];
        const double h = state_[idx_h_];
        const double r = earth_radius_ + h;
        return {
            r * std::cos(lat) * std::cos(lon),
            r * std::cos(lat) * std::sin(lon),
            r * std::sin(lat)
        };
    }

    double getAltitude() const override {
        return state_[idx_h_];
    }

    gnc::Vector3d getVelocity() const override {
        const double velocity = state_[idx_v_];
        const double gamma = state_[idx_gamma_];
        const double psi = state_[idx_psi_];
        return {
            velocity * std::cos(gamma) * std::cos(psi),
            velocity * std::cos(gamma) * std::sin(psi),
            -velocity * std::sin(gamma)
        };
    }

    void update(double) override {
    }

    std::vector<interfaces::ObservableField> getObservableFields() const override {
        core::ObservableFieldBuilder builder;
        builder.addScalar("longitude_deg", [this]() { return rad2deg(state_[idx_lon_]); });
        builder.addScalar("latitude_deg", [this]() { return rad2deg(state_[idx_lat_]); });
        builder.addScalar("altitude", [this]() { return state_[idx_h_]; });
        builder.addScalar("velocity", [this]() { return state_[idx_v_]; });
        builder.addScalar("flight_path_angle_deg", [this]() { return rad2deg(state_[idx_gamma_]); });
        builder.addScalar("heading_angle_deg", [this]() { return rad2deg(state_[idx_psi_]); });
        builder.addScalar("mach", [this]() {
            const double a = atmosphere_ ? atmosphere_->getSpeedOfSound(state_[idx_h_]) : 340.0;
            return state_[idx_v_] / std::max(1.0, a);
        });
        builder.addScalar("alpha_deg", [this]() {
            return guidance_ ? rad2deg(guidance_->getFlightCommand().alpha) : 0.0;
        });
        builder.addScalar("sigma_deg", [this]() {
            return guidance_ ? rad2deg(guidance_->getFlightCommand().sigma) : 0.0;
        });
        builder.addScalar("lift_to_drag", [this]() {
            if (!aero_) return 0.0;
            const double mach = atmosphere_ ? state_[idx_v_] / std::max(1.0, atmosphere_->getSpeedOfSound(state_[idx_h_])) : 0.0;
            const double alpha = guidance_ ? guidance_->getFlightCommand().alpha : 0.0;
            const auto coeffs = aero_->computeCoefficients(alpha, 0.0, mach);
            return std::abs(coeffs.CD) > 1e-9 ? coeffs.CL / coeffs.CD : 0.0;
        });
        builder.addScalar("timestamp", [this]() { return getSimTime(); });
        return builder.build();
    }

private:
    static double deg2rad(double deg) {
        return deg * 3.14159265358979323846 / 180.0;
    }

    static double rad2deg(double rad) {
        return rad * 180.0 / 3.14159265358979323846;
    }

    core::StateLayout layout_;
    Eigen::VectorXd state_;
    Eigen::VectorXd initial_state_;
    interfaces::IAtmosphereModel* atmosphere_ = nullptr;
    interfaces::IGravityModel* gravity_ = nullptr;
    interfaces::IAeroCoefficients* aero_ = nullptr;
    interfaces::IMassProperty* mass_ = nullptr;
    interfaces::IGuidance3DOF* guidance_ = nullptr;
    int idx_lon_ = -1;
    int idx_lat_ = -1;
    int idx_h_ = -1;
    int idx_v_ = -1;
    int idx_gamma_ = -1;
    int idx_psi_ = -1;
    double earth_radius_ = 6371000.0;
};

GNC_REGISTER_STARTER_COMPONENT(Dynamics3DOF_SphericalEarth,
                       interfaces::IDynamicsModel,
                       interfaces::IAltitudeProvider,
                       interfaces::IPositionProvider,
                       interfaces::IVelocityProvider)

} // namespace gnc::components
