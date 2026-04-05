/**
 * @file data_formatters.hpp
 * @brief 标准数据类型的格式化器
 * 
 * 为常用数据结构提供预置的 CSV 格式化函数
 */
#pragma once

#include "gnc/interfaces/data_types.hpp"
#include "gnc/common/math_types.hpp"
#include <sstream>
#include <iomanip>
#include <vector>
#include <string>

namespace gnc::core::formatters {

/// 设置浮点精度
constexpr int PRECISION = 12;

/// Vector3d 格式化
inline std::string formatVector3d(const Vector3d& v) {
    std::ostringstream oss;
    oss << std::setprecision(PRECISION) 
        << v.x << "," << v.y << "," << v.z;
    return oss.str();
}

/// Vector3d 字段名
inline std::vector<std::string> vector3dFields(const std::string& prefix) {
    return {prefix + ".x", prefix + ".y", prefix + ".z"};
}

/// Quaterniond 格式化
inline std::string formatQuaterniond(const Quaterniond& q) {
    std::ostringstream oss;
    oss << std::setprecision(PRECISION) 
        << q.w << "," << q.x << "," << q.y << "," << q.z;
    return oss.str();
}

/// Quaterniond 字段名
inline std::vector<std::string> quaterniondFields(const std::string& prefix) {
    return {prefix + ".w", prefix + ".x", prefix + ".y", prefix + ".z"};
}

/// ImuData 格式化
inline std::string formatImuData(const interfaces::ImuData& data) {
    std::ostringstream oss;
    oss << std::setprecision(PRECISION)
        << data.acceleration.x << "," 
        << data.acceleration.y << "," 
        << data.acceleration.z << ","
        << data.angular_velocity.x << "," 
        << data.angular_velocity.y << "," 
        << data.angular_velocity.z << ","
        << data.timestamp;
    return oss.str();
}

/// ImuData 字段名
inline std::vector<std::string> imuDataFields() {
    return {"acc.x", "acc.y", "acc.z", 
            "gyro.x", "gyro.y", "gyro.z", 
            "timestamp"};
}

/// NavState 格式化
inline std::string formatNavState(const interfaces::NavState& state) {
    std::ostringstream oss;
    oss << std::setprecision(PRECISION)
        << state.position.x << "," 
        << state.position.y << "," 
        << state.position.z << ","
        << state.velocity.x << "," 
        << state.velocity.y << "," 
        << state.velocity.z << ","
        << state.attitude.w << ","
        << state.attitude.x << ","
        << state.attitude.y << ","
        << state.attitude.z << ","
        << state.angular_velocity.x << "," 
        << state.angular_velocity.y << "," 
        << state.angular_velocity.z << ","
        << state.timestamp;
    return oss.str();
}

/// NavState 字段名
inline std::vector<std::string> navStateFields() {
    return {"pos.x", "pos.y", "pos.z",
            "vel.x", "vel.y", "vel.z",
            "att.w", "att.x", "att.y", "att.z",
            "omega.x", "omega.y", "omega.z",
            "timestamp"};
}

/// VehicleState 格式化
inline std::string formatVehicleState(const interfaces::VehicleState& state) {
    std::ostringstream oss;
    oss << std::setprecision(PRECISION)
        << state.position.x << "," 
        << state.position.y << "," 
        << state.position.z << ","
        << state.velocity.x << "," 
        << state.velocity.y << "," 
        << state.velocity.z << ","
        << state.attitude.w << ","
        << state.attitude.x << ","
        << state.attitude.y << ","
        << state.attitude.z << ","
        << state.angular_velocity.x << "," 
        << state.angular_velocity.y << "," 
        << state.angular_velocity.z << ","
        << state.timestamp;
    return oss.str();
}

/// VehicleState 字段名
inline std::vector<std::string> vehicleStateFields() {
    return {"pos.x", "pos.y", "pos.z",
            "vel.x", "vel.y", "vel.z",
            "att.w", "att.x", "att.y", "att.z",
            "omega.x", "omega.y", "omega.z",
            "timestamp"};
}

} // namespace gnc::core::formatters
