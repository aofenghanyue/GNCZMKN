/**
 * @file i_imu_sensor.hpp
 * @brief IMU传感器接口
 */
#pragma once

#include "gnc/interfaces/data_types.hpp"

namespace gnc::interfaces {

/**
 * @brief 惯性测量单元接口
 */
class IImuSensor {
public:
    virtual ~IImuSensor() = default;
    
    /// 获取IMU量测数据
    virtual const ImuData& getImuData() const = 0;
    
    /// 检查传感器是否正常工作
    virtual bool isHealthy() const = 0;
};

} // namespace gnc::interfaces
