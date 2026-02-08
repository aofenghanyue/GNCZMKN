/**
 * @file i_atmosphere_model.hpp
 * @brief 大气模型接口
 */
#pragma once

namespace gnc::interfaces {

/**
 * @brief 大气模型接口
 */
class IAtmosphereModel {
public:
    virtual ~IAtmosphereModel() = default;
    
    /// 获取指定高度的大气密度 (kg/m³)
    virtual double getDensity(double altitude) const = 0;
    
    /// 获取指定高度的大气压力 (Pa)
    virtual double getPressure(double altitude) const = 0;
    
    /// 获取指定高度的温度 (K)
    virtual double getTemperature(double altitude) const = 0;
    
    /// 获取指定高度的声速 (m/s)
    virtual double getSpeedOfSound(double altitude) const = 0;
};

} // namespace gnc::interfaces
