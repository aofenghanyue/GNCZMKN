/**
 * @file i_gps_sensor.hpp
 * @brief GPS传感器接口
 */
#pragma once

#include "gnc/interfaces/data_types.hpp"

namespace gnc::interfaces {

/**
 * @brief GPS接收机接口
 */
class IGpsSensor {
public:
    virtual ~IGpsSensor() = default;
    
    /// 获取GPS量测数据
    virtual const GpsData& getGpsData() const = 0;
    
    /// 检查是否有有效信号
    virtual bool hasValidFix() const = 0;
    
    /// 获取可见卫星数量
    virtual int getVisibleSatellites() const = 0;
};

} // namespace gnc::interfaces
