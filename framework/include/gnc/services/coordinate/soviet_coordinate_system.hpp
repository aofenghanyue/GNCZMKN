/**
 * @file soviet_coordinate_system.hpp
 * @brief Official entry point for the framework's built-in Soviet coordinate system.
 *
 * Public frame set:
 * - `I`  / `ECI`              : Earth-centered inertial frame.
 * - `E`  / `ECEF`             : Earth-centered Earth-fixed frame.
 * - `N`  / `NUE`              : Subject-local north-up-east frame rebuilt from current geodetic position.
 * - `L`  / `LAUNCH`           : Launch frame fixed at the configured launch site.
 * - `LI` / `LAUNCH_INERTIAL`  : Launch inertial frame, coincident with `L` at `t0`.
 * - `B`  / `BODY`             : Flight-vehicle body frame.
 * - `K`  / `TRACK`            : Track frame built from launch-frame ground velocity.
 * - `V`  / `WIND`             : Wind frame built from body-frame air-relative velocity.
 *
 * Conventions:
 * - All matrices are passive coordinate transforms and are registered as child->parent rotations.
 * - `N` is north-up-east. `L` is right-handed with +X along launch azimuth, +Y local up, +Z completing the triad.
 * - Launch azimuth is positive from north toward east about the local +Up axis.
 * - `B` relative to `L` follows the built-in 3-2-1 Euler convention exposed by `eulerToAttitude()` and
 *   `attitudeToEuler()`.
 * - `K` relative to `L` uses `chi = atan2(-v_z, v_x)` and `theta = atan2(v_y, sqrt(v_x^2 + v_z^2))`
 *   for a launch-frame ground-relative velocity vector.
 * - `V` relative to `B` uses `alpha = atan2(-v, u)` and `beta = asin(w / |V|)` for a body-frame
 *   air-relative velocity vector `(u, v, w)`.
 *
 * If your project uses different frame meanings, Euler definitions, rotation sequences, or sign conventions,
 * do not modify this built-in Soviet system. Define a separate coordinate system for your own conventions.
 */
#pragma once

#include "coordinate_service.hpp"
#include "gnc/common/math/math.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/interfaces/environment/i_earth_model.hpp"
#include "gnc/interfaces/state/i_attitude_provider.hpp"
#include "gnc/interfaces/state/i_position_provider.hpp"
#include "gnc/interfaces/state/i_velocity_provider.hpp"

#include <array>
#include <cctype>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace gnc::services::coordinate::soviet {

using gnc::coord::FrameId;
using gnc::coord::frameIdToString;
using gnc::math::Matrix3;
using gnc::math::Quaternion;
using gnc::math::Vector3;

inline constexpr double kVectorEpsilon = 1e-9;

struct LaunchFrameConfig {
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double azimuth_rad = 0.0;
    double t0 = 0.0;
    double earth_rotation_angle_rad = 0.0;
};

enum class SovietRelationId {
    EcefToEci,
    LaunchToEcef,
    LaunchInertialToLaunch,
    NueToEcef,
    BodyToLaunch,
    TrackToLaunch,
    WindToBody,
};

struct SovietInstallContext {
    const gnc::core::ConfigNode* config = nullptr;
    gnc::core::ScopedRegistry* registry = nullptr;
    std::string service_scope_name;
    std::string subject;
    LaunchFrameConfig launch;
    gnc::interfaces::IEarthModel* earth_model = nullptr;
    gnc::interfaces::IPositionProvider* local_geographic_provider = nullptr;
    gnc::interfaces::IAttitudeProvider* body_attitude_provider = nullptr;
    gnc::interfaces::IVelocityProvider* track_motion_provider = nullptr;
    gnc::interfaces::IVelocityProvider* wind_velocity_provider = nullptr;
};

struct SovietRelationSpec {
    SovietRelationId id;
    FrameId child;
    FrameId parent;
    const char* config_key;
    const char* physical_meaning;
    const char* input_semantics;
    bool (*enabled)(const SovietInstallContext&);
    void (*install)(CoordinateService&, const SovietRelationSpec&, const SovietInstallContext&);
};

inline std::string subjectPrefix(const std::string& subject) {
    return "Soviet coordinate bindings for flight vehicle/object '" + subject + "': ";
}

inline std::string joinKeys(const std::vector<std::string>& keys) {
    std::string joined;
    for (size_t i = 0; i < keys.size(); ++i) {
        if (i > 0) {
            joined += ", ";
        }
        joined += "'" + keys[i] + "'";
    }
    return joined;
}

inline void requireObject(const gnc::core::ConfigNode& node, const std::string& path) {
    if (!node.isObject()) {
        throw std::runtime_error(path + " must be an object.");
    }
}

inline void ensureOnlyKnownKeys(const gnc::core::ConfigNode& node,
                                const std::string& path,
                                std::initializer_list<const char*> allowed_keys) {
    requireObject(node, path);
    node.resetAccessTracking();
    for (const char* key : allowed_keys) {
        (void)node[key];
    }
    const auto unused = node.getUnusedKeys();
    if (!unused.empty()) {
        throw std::runtime_error(path + " contains unsupported field(s): " + joinKeys(unused) + ".");
    }
}

inline std::string requireBindingName(const gnc::core::ConfigNode& node, const std::string& path) {
    ensureOnlyKnownKeys(node, path, {"name"});
    const std::string name = node["name"].asString();
    if (name.empty()) {
        throw std::runtime_error(path + ".name must be a non-empty provider name.");
    }
    return name;
}

template<typename Interface>
Interface* requireBinding(gnc::core::ScopedRegistry& registry,
                          const gnc::core::ConfigNode& bindings,
                          const char* key,
                          const std::string& root_path) {
    const auto& block = bindings[key];
    if (block.isNull()) {
        throw std::runtime_error(root_path + "." + key + " is required by the built-in Soviet coordinate system.");
    }
    return registry.requireByName<Interface>(requireBindingName(block, root_path + "." + key));
}

template<typename Interface>
Interface* tryBinding(gnc::core::ScopedRegistry& registry,
                      const gnc::core::ConfigNode& bindings,
                      const char* key,
                      const std::string& root_path) {
    const auto& block = bindings[key];
    if (block.isNull()) {
        return nullptr;
    }
    return registry.requireByName<Interface>(requireBindingName(block, root_path + "." + key));
}

inline bool isGlobalScope(const std::string& scope_name) {
    return scope_name == "global";
}

inline bool isEnvironmentScope(const std::string& scope_name) {
    return scope_name == "env";
}

inline std::string defaultSubjectForScope(const std::string& scope_name) {
    if (scope_name.empty()) {
        return "flight_vehicle";
    }
    return scope_name;
}

inline std::string normalizeSchemeName(std::string scheme) {
    for (char& ch : scheme) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return scheme;
}

inline bool hasDynamicBindingRequest(const gnc::core::ConfigNode& bindings) {
    return bindings.has("local_geographic") ||
           bindings.has("body_attitude") ||
           bindings.has("track_motion") ||
           bindings.has("wind_velocity");
}

inline LaunchFrameConfig readLaunchFrameConfig(const gnc::core::ConfigNode& launch_config,
                                               const std::string& path) {
    ensureOnlyKnownKeys(launch_config, path, {
        "latitude_rad",
        "longitude_rad",
        "azimuth_rad",
        "t0",
        "earth_rotation_angle_rad"
    });

    LaunchFrameConfig launch;
    launch.latitude_rad = launch_config["latitude_rad"].asDouble();
    launch.longitude_rad = launch_config["longitude_rad"].asDouble();
    launch.azimuth_rad = launch_config["azimuth_rad"].asDouble();
    launch.t0 = launch_config["t0"].asDouble(0.0);
    launch.earth_rotation_angle_rad = launch_config["earth_rotation_angle_rad"].asDouble(0.0);
    return launch;
}

inline std::string resolveSubject(const gnc::core::ConfigNode& bindings,
                                  const std::string& service_scope_name) {
    const auto& subject_node = bindings["subject"];
    const std::string subject = subject_node.asString();
    if (!subject.empty()) {
        return subject;
    }

    if (isGlobalScope(service_scope_name) || isEnvironmentScope(service_scope_name)) {
        throw std::runtime_error(
            "Dynamic Soviet coordinate bindings in scope '" + service_scope_name +
            "' require an explicit bindings.subject.");
    }

    return defaultSubjectForScope(service_scope_name);
}

inline Vector3 toMathVector(const gnc::Vector3d& value) {
    return Vector3(value.x, value.y, value.z);
}

inline Quaternion toMathQuaternion(const gnc::Quaterniond& value) {
    return Quaternion(value.w, value.x, value.y, value.z);
}

inline Matrix3 inertialToEarthFixedRotation(double earth_rotation_angle_rad) {
    const double c = std::cos(earth_rotation_angle_rad);
    const double s = std::sin(earth_rotation_angle_rad);

    Matrix3 rotation;
    rotation <<  c, s, 0.0,
                -s, c, 0.0,
               0.0, 0.0, 1.0;
    return rotation;
}

inline Matrix3 earthFixedToInertialRotation(double earth_rotation_angle_rad) {
    return inertialToEarthFixedRotation(earth_rotation_angle_rad).transpose();
}

inline Matrix3 earthFixedToLocalNueRotation(double latitude_rad, double longitude_rad) {
    const double sin_lat = std::sin(latitude_rad);
    const double cos_lat = std::cos(latitude_rad);
    const double sin_lon = std::sin(longitude_rad);
    const double cos_lon = std::cos(longitude_rad);

    Matrix3 rotation;
    rotation << -sin_lat * cos_lon, -sin_lat * sin_lon,  cos_lat,
                 cos_lat * cos_lon,  cos_lat * sin_lon,  sin_lat,
                -sin_lon,            cos_lon,            0.0;
    return rotation;
}

inline Matrix3 localNueToEarthFixedRotation(double latitude_rad, double longitude_rad) {
    return earthFixedToLocalNueRotation(latitude_rad, longitude_rad).transpose();
}

inline Matrix3 localNueToLaunchRotation(double azimuth_rad) {
    const double c = std::cos(azimuth_rad);
    const double s = std::sin(azimuth_rad);

    Matrix3 rotation;
    rotation <<  c, 0.0,  s,
                0.0, 1.0, 0.0,
                -s, 0.0,  c;
    return rotation;
}

inline Matrix3 launchToLocalNueRotation(double azimuth_rad) {
    return localNueToLaunchRotation(azimuth_rad).transpose();
}

inline Matrix3 earthFixedToLaunchRotation(double latitude_rad,
                                          double longitude_rad,
                                          double azimuth_rad) {
    return localNueToLaunchRotation(azimuth_rad) *
           earthFixedToLocalNueRotation(latitude_rad, longitude_rad);
}

inline Matrix3 launchToEarthFixedRotation(double latitude_rad,
                                          double longitude_rad,
                                          double azimuth_rad) {
    return earthFixedToLaunchRotation(latitude_rad, longitude_rad, azimuth_rad).transpose();
}

inline Matrix3 launchInertialToLaunchRotation(double delta_t,
                                              const Matrix3& earth_fixed_to_launch,
                                              double earth_rotation_rate_rad_s) {
    return earth_fixed_to_launch *
           inertialToEarthFixedRotation(earth_rotation_rate_rad_s * delta_t) *
           earth_fixed_to_launch.transpose();
}

inline Matrix3 referenceToBodyRotation(const Quaternion& attitude_reference_to_body) {
    return attitude_reference_to_body.toRotationMatrix();
}

inline Matrix3 bodyToReferenceRotation(const Quaternion& attitude_reference_to_body) {
    return referenceToBodyRotation(attitude_reference_to_body).transpose();
}

inline Matrix3 referenceToTrackRotation(double chi_rad, double theta_rad) {
    const double c_chi = std::cos(chi_rad);
    const double s_chi = std::sin(chi_rad);
    const double c_theta = std::cos(theta_rad);
    const double s_theta = std::sin(theta_rad);

    Matrix3 rotation;
    rotation <<  c_theta * c_chi,  s_theta, -c_theta * s_chi,
                -s_theta * c_chi,  c_theta,  s_theta * s_chi,
                 s_chi,            0.0,      c_chi;
    return rotation;
}

inline Matrix3 trackToReferenceRotation(double chi_rad, double theta_rad) {
    return referenceToTrackRotation(chi_rad, theta_rad).transpose();
}

inline Matrix3 windToBodyRotation(double alpha_rad, double beta_rad) {
    const double c_alpha = std::cos(alpha_rad);
    const double s_alpha = std::sin(alpha_rad);
    const double c_beta = std::cos(beta_rad);
    const double s_beta = std::sin(beta_rad);

    Matrix3 rotation;
    rotation <<  c_alpha * c_beta,  s_alpha, -c_alpha * s_beta,
                -s_alpha * c_beta,  c_alpha,  s_alpha * s_beta,
                 s_beta,            0.0,      c_beta;
    return rotation;
}

inline Matrix3 bodyToWindRotation(double alpha_rad, double beta_rad) {
    return windToBodyRotation(alpha_rad, beta_rad).transpose();
}

inline void computeTrackAngles(const Vector3& ground_velocity_launch,
                               double& chi_rad,
                               double& theta_rad) {
    const double v_x = ground_velocity_launch.x();
    const double v_y = ground_velocity_launch.y();
    const double v_z = ground_velocity_launch.z();
    const double horizontal_speed = std::sqrt(v_x * v_x + v_z * v_z);

    chi_rad = std::atan2(-v_z, v_x);
    theta_rad = std::atan2(v_y, horizontal_speed);
}

inline void computeWindAngles(const Vector3& air_velocity_body,
                              double& alpha_rad,
                              double& beta_rad) {
    const double u = air_velocity_body.x();
    const double v = air_velocity_body.y();
    const double w = air_velocity_body.z();
    const double speed = air_velocity_body.norm();

    if (speed < kVectorEpsilon) {
        alpha_rad = 0.0;
        beta_rad = 0.0;
        return;
    }

    alpha_rad = std::atan2(-v, u);
    beta_rad = std::asin(w / speed);
}

inline Quaternion eulerToAttitude(double yaw_rad, double pitch_rad, double roll_rad) {
    return gnc::math::euler321ToQuat(yaw_rad, pitch_rad, roll_rad);
}

inline Vector3 attitudeToEuler(const Quaternion& attitude_reference_to_body) {
    const Matrix3 rotation = attitude_reference_to_body.toRotationMatrix();
    const double pitch_rad = std::asin(-rotation(0, 2));

    double yaw_rad = 0.0;
    double roll_rad = 0.0;
    if (std::abs(std::cos(pitch_rad)) > 1e-10) {
        yaw_rad = std::atan2(rotation(0, 1), rotation(0, 0));
        roll_rad = std::atan2(rotation(1, 2), rotation(2, 2));
    } else {
        roll_rad = std::atan2(-rotation(1, 0), rotation(1, 1));
    }
    return Vector3(yaw_rad, pitch_rad, roll_rad);
}

inline void readLocalGeodetic(const SovietInstallContext& context,
                              double& latitude_rad,
                              double& longitude_rad,
                              double& altitude_m) {
    if (!context.local_geographic_provider) {
        throw std::runtime_error(subjectPrefix(context.subject) +
                                 "dynamic NUE requires bindings.local_geographic.name.");
    }

    const gnc::Vector3d position = context.local_geographic_provider->getPosition();
    latitude_rad = position.x;
    longitude_rad = position.y;
    altitude_m = position.z;
}

inline bool alwaysEnabled(const SovietInstallContext&) { return true; }
inline bool hasDynamicNue(const SovietInstallContext& context) { return context.local_geographic_provider != nullptr; }
inline bool hasBody(const SovietInstallContext& context) { return context.body_attitude_provider != nullptr; }
inline bool hasTrack(const SovietInstallContext& context) { return context.track_motion_provider != nullptr; }
inline bool hasWind(const SovietInstallContext& context) { return context.wind_velocity_provider != nullptr; }

inline void installEcefToEciRelation(CoordinateService& service,
                                     const SovietRelationSpec&,
                                     const SovietInstallContext& context) {
    if (!context.earth_model) {
        throw std::runtime_error("The built-in Soviet coordinate system requires bindings.earth.name.");
    }

    const auto launch = context.launch;
    auto* earth_model = context.earth_model;
    service.registerTransform(FrameId::ECEF, FrameId::ECI, [launch, earth_model](double time) {
        const double angle = launch.earth_rotation_angle_rad + earth_model->getRotationRate() * time;
        return earthFixedToInertialRotation(angle);
    });
}

inline void installLaunchToEcefRelation(CoordinateService& service,
                                        const SovietRelationSpec&,
                                        const SovietInstallContext& context) {
    const auto launch = context.launch;
    service.registerTransform(FrameId::LAUNCH, FrameId::ECEF, [launch]() {
        return launchToEarthFixedRotation(launch.latitude_rad, launch.longitude_rad, launch.azimuth_rad);
    });
}

inline void installLaunchInertialToLaunchRelation(CoordinateService& service,
                                                  const SovietRelationSpec&,
                                                  const SovietInstallContext& context) {
    if (!context.earth_model) {
        throw std::runtime_error("The built-in Soviet coordinate system requires bindings.earth.name.");
    }

    const auto launch = context.launch;
    const Matrix3 earth_fixed_to_launch =
        earthFixedToLaunchRotation(launch.latitude_rad, launch.longitude_rad, launch.azimuth_rad);
    auto* earth_model = context.earth_model;
    service.registerTransform(FrameId::LAUNCH_INERTIAL, FrameId::LAUNCH, [launch, earth_fixed_to_launch, earth_model](double time) {
        return launchInertialToLaunchRotation(
            time - launch.t0,
            earth_fixed_to_launch,
            earth_model->getRotationRate());
    });
}

inline void installNueToEcefRelation(CoordinateService& service,
                                     const SovietRelationSpec&,
                                     const SovietInstallContext& context) {
    auto* position_provider = context.local_geographic_provider;
    const std::string subject = context.subject;
    service.registerTransform(FrameId::NUE, FrameId::ECEF, [position_provider, subject](double) {
        if (!position_provider) {
            throw std::runtime_error(subjectPrefix(subject) +
                                     "dynamic NUE requires bindings.local_geographic.name.");
        }

        const gnc::Vector3d geodetic = position_provider->getPosition();
        return localNueToEarthFixedRotation(geodetic.x, geodetic.y);
    });
}

inline void installBodyToLaunchRelation(CoordinateService& service,
                                        const SovietRelationSpec&,
                                        const SovietInstallContext& context) {
    auto* attitude_provider = context.body_attitude_provider;
    const std::string subject = context.subject;
    service.registerTransform(FrameId::BODY, FrameId::LAUNCH, [attitude_provider, subject](double) {
        if (!attitude_provider) {
            throw std::runtime_error(subjectPrefix(subject) +
                                     "BODY requires bindings.body_attitude.name.");
        }

        const Quaternion attitude_launch_to_body = toMathQuaternion(attitude_provider->getAttitude());
        return bodyToReferenceRotation(attitude_launch_to_body);
    });
}

inline void installTrackToLaunchRelation(CoordinateService& service,
                                         const SovietRelationSpec&,
                                         const SovietInstallContext& context) {
    auto* velocity_provider = context.track_motion_provider;
    const std::string subject = context.subject;
    service.registerTransform(FrameId::TRACK, FrameId::LAUNCH, [velocity_provider, subject](double) {
        if (!velocity_provider) {
            throw std::runtime_error(subjectPrefix(subject) +
                                     "TRACK requires bindings.track_motion.name.");
        }

        const Vector3 ground_velocity_launch = toMathVector(velocity_provider->getVelocity());
        if (ground_velocity_launch.norm() < kVectorEpsilon) {
            throw std::runtime_error(subjectPrefix(subject) +
                                     "TRACK is undefined when launch-frame ground speed is near zero.");
        }

        double chi_rad = 0.0;
        double theta_rad = 0.0;
        computeTrackAngles(ground_velocity_launch, chi_rad, theta_rad);
        return trackToReferenceRotation(chi_rad, theta_rad);
    });
}

inline void installWindToBodyRelation(CoordinateService& service,
                                      const SovietRelationSpec&,
                                      const SovietInstallContext& context) {
    auto* velocity_provider = context.wind_velocity_provider;
    const std::string subject = context.subject;
    service.registerTransform(FrameId::WIND, FrameId::BODY, [velocity_provider, subject](double) {
        if (!velocity_provider) {
            throw std::runtime_error(subjectPrefix(subject) +
                                     "WIND requires bindings.wind_velocity.name.");
        }

        const Vector3 air_velocity_body = toMathVector(velocity_provider->getVelocity());
        if (air_velocity_body.norm() < kVectorEpsilon) {
            throw std::runtime_error(subjectPrefix(subject) +
                                     "WIND is undefined when body-frame airspeed is near zero.");
        }

        double alpha_rad = 0.0;
        double beta_rad = 0.0;
        computeWindAngles(air_velocity_body, alpha_rad, beta_rad);
        return windToBodyRotation(alpha_rad, beta_rad);
    });
}

inline const std::array<SovietRelationSpec, 7>& sovietRelationSpecs() {
    static const std::array<SovietRelationSpec, 7> specs{{
        {
            SovietRelationId::EcefToEci,
            FrameId::ECEF,
            FrameId::ECI,
            "bindings.earth",
            "Earth-fixed frame relative to inertial frame.",
            "IEarthModel rotation rate and launch.earth_rotation_angle_rad.",
            alwaysEnabled,
            installEcefToEciRelation
        },
        {
            SovietRelationId::LaunchToEcef,
            FrameId::LAUNCH,
            FrameId::ECEF,
            "launch",
            "Launch frame fixed at the configured launch site.",
            "launch.latitude_rad, launch.longitude_rad, launch.azimuth_rad.",
            alwaysEnabled,
            installLaunchToEcefRelation
        },
        {
            SovietRelationId::LaunchInertialToLaunch,
            FrameId::LAUNCH_INERTIAL,
            FrameId::LAUNCH,
            "launch",
            "Launch inertial frame coincident with launch frame at t0.",
            "launch.t0 plus IEarthModel rotation rate.",
            alwaysEnabled,
            installLaunchInertialToLaunchRelation
        },
        {
            SovietRelationId::NueToEcef,
            FrameId::NUE,
            FrameId::ECEF,
            "bindings.local_geographic",
            "Dynamic local north-up-east frame for the bound subject.",
            "IPositionProvider returning [lat_rad, lon_rad, alt_m].",
            hasDynamicNue,
            installNueToEcefRelation
        },
        {
            SovietRelationId::BodyToLaunch,
            FrameId::BODY,
            FrameId::LAUNCH,
            "bindings.body_attitude",
            "Flight-vehicle body frame relative to launch frame.",
            "IAttitudeProvider returning the launch-to-body attitude quaternion q_L^B.",
            hasBody,
            installBodyToLaunchRelation
        },
        {
            SovietRelationId::TrackToLaunch,
            FrameId::TRACK,
            FrameId::LAUNCH,
            "bindings.track_motion",
            "Track frame relative to launch frame.",
            "IVelocityProvider returning launch-frame ground-relative velocity.",
            hasTrack,
            installTrackToLaunchRelation
        },
        {
            SovietRelationId::WindToBody,
            FrameId::WIND,
            FrameId::BODY,
            "bindings.wind_velocity",
            "Wind frame relative to body frame.",
            "IVelocityProvider returning body-frame air-relative velocity.",
            hasWind,
            installWindToBodyRelation
        }
    }};
    return specs;
}

inline void installSovietRelation(CoordinateService& service,
                                  const SovietRelationSpec& relation_spec,
                                  const SovietInstallContext& context) {
    if (!relation_spec.enabled || relation_spec.enabled(context)) {
        relation_spec.install(service, relation_spec, context);
    }
}

inline SovietInstallContext buildInstallContext(const gnc::core::ConfigNode& coordinate_config,
                                                gnc::core::ScopedRegistry& registry,
                                                const std::string& service_scope_name) {
    ensureOnlyKnownKeys(coordinate_config, "services.coordinate", {
        "enabled",
        "scheme",
        "launch",
        "bindings"
    });

    const auto& launch_config = coordinate_config["launch"];
    if (launch_config.isNull()) {
        throw std::runtime_error("services.coordinate.launch is required by the built-in Soviet coordinate system.");
    }

    const auto& bindings = coordinate_config["bindings"];
    if (bindings.isNull()) {
        throw std::runtime_error("services.coordinate.bindings is required by the built-in Soviet coordinate system.");
    }

    ensureOnlyKnownKeys(bindings, "services.coordinate.bindings", {
        "subject",
        "earth",
        "local_geographic",
        "body_attitude",
        "track_motion",
        "wind_velocity"
    });

    SovietInstallContext context;
    context.config = &coordinate_config;
    context.registry = &registry;
    context.service_scope_name = service_scope_name;
    context.launch = readLaunchFrameConfig(launch_config, "services.coordinate.launch");
    context.earth_model = requireBinding<gnc::interfaces::IEarthModel>(
        registry,
        bindings,
        "earth",
        "services.coordinate.bindings");

    const bool dynamic_requested = hasDynamicBindingRequest(bindings);
    if (dynamic_requested) {
        context.subject = resolveSubject(bindings, service_scope_name);
    }

    context.local_geographic_provider = tryBinding<gnc::interfaces::IPositionProvider>(
        registry,
        bindings,
        "local_geographic",
        "services.coordinate.bindings");
    context.body_attitude_provider = tryBinding<gnc::interfaces::IAttitudeProvider>(
        registry,
        bindings,
        "body_attitude",
        "services.coordinate.bindings");
    context.track_motion_provider = tryBinding<gnc::interfaces::IVelocityProvider>(
        registry,
        bindings,
        "track_motion",
        "services.coordinate.bindings");
    context.wind_velocity_provider = tryBinding<gnc::interfaces::IVelocityProvider>(
        registry,
        bindings,
        "wind_velocity",
        "services.coordinate.bindings");

    return context;
}

inline void installSovietCoordinateSystem(CoordinateService& service,
                                          const gnc::core::ConfigNode& coordinate_config,
                                          gnc::core::ScopedRegistry& registry,
                                          const std::string& service_scope_name) {
    const std::string scheme = normalizeSchemeName(coordinate_config["scheme"].asString("soviet"));
    if (!scheme.empty() && scheme != "soviet") {
        throw std::runtime_error(
            "Unknown coordinate service scheme '" + scheme + "'. Only 'soviet' is supported.");
    }

    service.clear();
    const SovietInstallContext context = buildInstallContext(coordinate_config, registry, service_scope_name);
    for (const auto& relation_spec : sovietRelationSpecs()) {
        installSovietRelation(service, relation_spec, context);
    }
}

} // namespace gnc::services::coordinate::soviet
