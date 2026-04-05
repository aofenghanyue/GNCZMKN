/**
 * @file types.hpp
 * @brief 基础类型定义
 */
#pragma once

#include <cstdint>
#include <string>

namespace gnc {

/// 飞行器ID类型
using VehicleId = uint32_t;

/// 组件ID：飞行器ID + 组件名称
struct ComponentId {
    VehicleId vehicle_id;
    std::string component_name;
    
    bool operator==(const ComponentId& other) const {
        return vehicle_id == other.vehicle_id && 
               component_name == other.component_name;
    }
};

/// 仿真时间类型 (秒)
using SimTime = double;

} // namespace gnc
