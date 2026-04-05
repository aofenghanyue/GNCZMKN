/**
 * @file data_types.hpp
 * @brief 标准数据包定义
 * 
 * 这些是各接口间传递的标准数据结构
 */
#pragma once

#include "gnc/common/math_types.hpp"
#include <vector>

namespace gnc::interfaces {

/// IMU量测数据包
struct ImuData {
    Vector3d acceleration;      ///< 加速度 (m/s²)
    Vector3d angular_velocity;  ///< 角速度 (rad/s)
    double timestamp = 0.0;     ///< 时间戳 (s)
};

/// GPS量测数据包
struct GpsData {
    Vector3d position;          ///< 位置 (m, ECEF或本地坐标)
    Vector3d velocity;          ///< 速度 (m/s)
    double timestamp = 0.0;
    bool valid = false;         ///< 数据有效标志
};

/// 导航状态数据包
struct NavState {
    Vector3d position;          ///< 位置 (m)
    Vector3d velocity;          ///< 速度 (m/s)
    Quaterniond attitude;       ///< 姿态四元数
    Vector3d angular_velocity;  ///< 机体角速度 (rad/s)
    double timestamp = 0.0;
};

/// 制导指令数据包
struct GuidanceCommand {
    Vector3d acceleration_cmd;  ///< 期望加速度 (m/s²)
    Vector3d attitude_cmd;      ///< 期望姿态 (欧拉角, rad)
    double timestamp = 0.0;
};

/// 控制指令数据包
struct ControlCommand {
    Vector3d torque_cmd;        ///< 力矩指令 (Nm)
    Vector3d force_cmd;         ///< 力指令 (N)
    double timestamp = 0.0;
};

/// 执行器指令数据包
struct ActuatorCommand {
    std::vector<double> commands;  ///< 执行器指令列表
    double timestamp = 0.0;
};

/// 飞行器状态（动力学输出）
struct VehicleState {
    Vector3d position;          ///< 位置 (m)
    Vector3d velocity;          ///< 速度 (m/s)
    Quaterniond attitude;       ///< 姿态四元数
    Vector3d angular_velocity;  ///< 角速度 (rad/s)
    double timestamp = 0.0;
};

/// 扰动数据包
struct DisturbanceData {
    Vector3d force;             ///< 扰动力 (N)
    Vector3d torque;            ///< 扰动力矩 (Nm)
};

} // namespace gnc::interfaces
