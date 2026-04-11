#pragma once

#include "gnc/common/math_types.hpp"
#include "gnc/interfaces/data_types.hpp"

#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace gnc::core::formatters {

constexpr int PRECISION = 12;

inline std::string formatVector3d(const Vector3d& vector) {
    std::ostringstream oss;
    oss << std::setprecision(PRECISION)
        << vector.x << "," << vector.y << "," << vector.z;
    return oss.str();
}

inline std::vector<std::string> vector3dFields(const std::string& prefix) {
    return {prefix + ".x", prefix + ".y", prefix + ".z"};
}

inline std::string formatQuaterniond(const Quaterniond& quaternion) {
    std::ostringstream oss;
    oss << std::setprecision(PRECISION)
        << quaternion.w << "," << quaternion.x << ","
        << quaternion.y << "," << quaternion.z;
    return oss.str();
}

inline std::vector<std::string> quaterniondFields(const std::string& prefix) {
    return {prefix + ".w", prefix + ".x", prefix + ".y", prefix + ".z"};
}

inline std::string formatImuData(const interfaces::ImuData& data) {
    std::ostringstream oss;
    oss << std::setprecision(PRECISION)
        << data.acceleration.x << "," << data.acceleration.y << ","
        << data.acceleration.z << "," << data.angular_velocity.x << ","
        << data.angular_velocity.y << "," << data.angular_velocity.z << ","
        << data.timestamp;
    return oss.str();
}

inline std::vector<std::string> imuDataFields() {
    return {"acc.x", "acc.y", "acc.z", "gyro.x", "gyro.y", "gyro.z", "timestamp"};
}

inline std::string formatGpsData(const interfaces::GpsData& data) {
    std::ostringstream oss;
    oss << std::setprecision(PRECISION)
        << data.position.x << "," << data.position.y << "," << data.position.z << ","
        << data.velocity.x << "," << data.velocity.y << "," << data.velocity.z << ","
        << data.timestamp << "," << (data.valid ? 1 : 0);
    return oss.str();
}

inline std::vector<std::string> gpsDataFields() {
    return {"pos.x", "pos.y", "pos.z", "vel.x", "vel.y", "vel.z", "timestamp", "valid"};
}

inline std::string formatActuatorCommand(const interfaces::ActuatorCommand& command) {
    std::ostringstream oss;
    oss << std::setprecision(PRECISION);
    for (size_t i = 0; i < command.commands.size(); ++i) {
        if (i > 0) {
            oss << ",";
        }
        oss << command.commands[i];
    }
    if (!command.commands.empty()) {
        oss << ",";
    }
    oss << command.timestamp;
    return oss.str();
}

inline std::vector<std::string> actuatorCommandFields(size_t actuator_count) {
    std::vector<std::string> fields;
    fields.reserve(actuator_count + 1);
    for (size_t i = 0; i < actuator_count; ++i) {
        fields.push_back("command_" + std::to_string(i));
    }
    fields.push_back("timestamp");
    return fields;
}

inline std::string formatDisturbanceData(const interfaces::DisturbanceData& data) {
    std::ostringstream oss;
    oss << std::setprecision(PRECISION)
        << data.force.x << "," << data.force.y << "," << data.force.z << ","
        << data.torque.x << "," << data.torque.y << "," << data.torque.z;
    return oss.str();
}

inline std::vector<std::string> disturbanceDataFields() {
    return {"force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"};
}

} // namespace gnc::core::formatters
