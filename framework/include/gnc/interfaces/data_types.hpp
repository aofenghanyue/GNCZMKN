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



/// 执行器指令数据包
struct ActuatorCommand {
    std::vector<double> commands;  ///< 执行器指令列表
    double timestamp = 0.0;
};

/// 扰动数据包
struct DisturbanceData {
    Vector3d force;             ///< 扰动力 (N)
    Vector3d torque;            ///< 扰动力矩 (Nm)
};

} // namespace gnc::interfaces
