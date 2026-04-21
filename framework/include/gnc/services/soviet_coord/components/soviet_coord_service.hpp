#pragma once

#include "gnc/core/scoped_registry.hpp"
#include "gnc/environment/interfaces/i_earth.hpp"
#include "gnc/services/soviet_coord/interfaces/i_body_airspeed_provider.hpp"
#include "gnc/services/soviet_coord/interfaces/i_body_attitude_provider.hpp"
#include "gnc/services/soviet_coord/interfaces/i_coord_service.hpp"
#include "gnc/services/soviet_coord/interfaces/i_velocity_direction_provider.hpp"
#include "gnc/services/soviet_coord/internal/coordinate_tree.hpp"
#include "gnc/services/soviet_coord/internal/frame_ids.hpp"
#include "gnc/services/soviet_coord/internal/rotation_formulas.hpp"

#include <stdexcept>
#include <string>

namespace gnc::services::soviet_coord {

struct SovietCoordLaunchConfig {
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double azimuth_rad = 0.0;
    double launch_time_s = 0.0;
    double earth_rotation_angle_rad = 0.0;
};

struct SovietCoordBindings {
    std::string earth;
    std::string velocity_direction;
    std::string body_attitude;
    std::string body_airspeed;
};

class SovietCoordService final : public ICoordService {
public:
    SovietCoordService(SovietCoordLaunchConfig launch_config,
                       SovietCoordBindings bindings)
        : launch_config_(launch_config), bindings_(std::move(bindings)) {}

    void bindProviders(gnc::core::ScopedRegistry& registry) {
        earth_ = registry.requireByName<gnc::environment::IEarth>(bindings_.earth);
        if (!bindings_.velocity_direction.empty()) {
            velocity_direction_provider_ =
                registry.requireByName<IVelocityDirectionProvider>(bindings_.velocity_direction);
        }
        if (!bindings_.body_attitude.empty()) {
            body_attitude_provider_ =
                registry.requireByName<IBodyAttitudeProvider>(bindings_.body_attitude);
        }
        if (!bindings_.body_airspeed.empty()) {
            body_airspeed_provider_ =
                registry.requireByName<IBodyAirspeedProvider>(bindings_.body_airspeed);
        }
        installCoordinateSystems();
    }

    gnc::math::Vector3 transform(const gnc::math::Vector3& vector,
                                 const std::string& from_frame,
                                 const std::string& to_frame,
                                 double time) const override {
        return tree_.transform(vector, from_frame, to_frame, time);
    }

    gnc::math::Matrix3 getRotation(const std::string& from_frame,
                                   const std::string& to_frame,
                                   double time) const override {
        return tree_.getRotation(from_frame, to_frame, time);
    }

    bool hasFrame(const std::string& frame_id) const override {
        return tree_.hasFrame(frame_id);
    }

    bool hasEdge(const std::string& child_frame,
                 const std::string& parent_frame) const override {
        return tree_.hasEdge(child_frame, parent_frame);
    }

private:
    void installCoordinateSystems() {
        if (!earth_) {
            throw std::runtime_error("Soviet coordinate service requires an IEarth binding.");
        }

        tree_.setRoot(frames::kInertial);
        tree_.registerEdge(
            frames::kEarthFixed,
            frames::kInertial,
            [this](double time) {
                return internal::ecefToInertialRotation(
                    launch_config_.earth_rotation_angle_rad +
                    earth_->getRotationRate() * time);
            },
            false);
        tree_.registerEdge(
            frames::kLocalGeographic,
            frames::kEarthFixed,
            [this](double) {
                return internal::localGeographicToEarthFixedRotation(
                    launch_config_.latitude_rad,
                    launch_config_.longitude_rad);
            },
            true);
        tree_.registerEdge(
            frames::kLaunch,
            frames::kLocalGeographic,
            [this](double) {
                return internal::launchToLocalGeographicRotation(
                    launch_config_.azimuth_rad);
            },
            true);
        tree_.registerEdge(
            frames::kLaunchInertial,
            frames::kLaunch,
            [this](double time) {
                return internal::launchInertialToLaunchRotation(
                    launch_config_.launch_time_s,
                    time,
                    earth_->getRotationRate());
            },
            false);

        if (velocity_direction_provider_) {
            tree_.registerEdge(
                frames::kTrack,
                frames::kLaunch,
                [this](double) {
                    return internal::trackToLaunchRotation(
                        velocity_direction_provider_->getVelocityInLaunchFrame());
                },
                false);
        }
        if (body_attitude_provider_) {
            tree_.registerEdge(
                frames::kBody,
                frames::kLaunch,
                [this](double) {
                    return body_attitude_provider_->getBodyToLaunchRotation();
                },
                false);
        }
        if (body_attitude_provider_ && body_airspeed_provider_) {
            tree_.registerEdge(
                frames::kAirflow,
                frames::kBody,
                [this](double) {
                    return internal::airflowToBodyRotation(
                        body_airspeed_provider_->getAirspeedInBodyFrame());
                },
                false);
        }
    }

    SovietCoordLaunchConfig launch_config_;
    SovietCoordBindings bindings_;
    internal::CoordinateTree tree_;
    gnc::environment::IEarth* earth_ = nullptr;
    IVelocityDirectionProvider* velocity_direction_provider_ = nullptr;
    IBodyAttitudeProvider* body_attitude_provider_ = nullptr;
    IBodyAirspeedProvider* body_airspeed_provider_ = nullptr;
};

} // namespace gnc::services::soviet_coord
