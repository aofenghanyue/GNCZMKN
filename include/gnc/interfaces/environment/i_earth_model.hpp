/**
 * @file i_earth_model.hpp
 * @brief 地球模型接口
 */
#pragma once

#include "gnc/common/math_types.hpp"

namespace gnc::interfaces {

/**
 * @brief 地球模型接口
 */
class IEarthModel {
public:
    virtual ~IEarthModel() = default;
    
    /// 获取地球半径 (m)
    virtual double getRadius() const = 0;
    
    /// 获取地球自转角速度 (rad/s)
    virtual double getRotationRate() const = 0;
    
    /// 获取扁率
    virtual double getFlattening() const = 0;
    
    /// 将经纬高转换为ECEF坐标
    virtual Vector3d geodeticToEcef(double lat, double lon, double alt) const = 0;
    
    /// 将ECEF坐标转换为经纬高
    virtual void ecefToGeodetic(const Vector3d& ecef, 
                                 double& lat, double& lon, double& alt) const = 0;
};

} // namespace gnc::interfaces
