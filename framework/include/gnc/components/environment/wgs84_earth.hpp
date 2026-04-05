/**
 * @file wgs84_earth.hpp
 * @brief WGS84地球模型实现
 */
#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/interfaces/environment/i_earth_model.hpp"
#include <cmath>

namespace gnc::components {

/**
 * @brief WGS84地球模型
 */
class Wgs84Earth : public core::ComponentBase,
                   public interfaces::IEarthModel {
public:
    Wgs84Earth() : ComponentBase("Wgs84Earth") {}
    
    // --- IEarthModel 接口实现 ---
    
    double getRadius() const override {
        return 6378137.0;  // WGS84 长半轴 (m)
    }
    
    double getRotationRate() const override {
        return 7.292115e-5;  // rad/s
    }
    
    double getFlattening() const override {
        return 1.0 / 298.257223563;
    }
    
    Vector3d geodeticToEcef(double lat, double lon, double alt) const override {
        // TODO: 实现完整的坐标转换
        double a = getRadius();
        double coslat = std::cos(lat);
        double sinlat = std::sin(lat);
        double coslon = std::cos(lon);
        double sinlon = std::sin(lon);
        
        double N = a;  // 简化：忽略扁率
        
        return Vector3d{
            (N + alt) * coslat * coslon,
            (N + alt) * coslat * sinlon,
            (N + alt) * sinlat
        };
    }
    
    void ecefToGeodetic(const Vector3d& ecef,
                        double& lat, double& lon, double& alt) const override {
        // TODO: 实现完整的坐标转换
        double x = ecef.x, y = ecef.y, z = ecef.z;
        
        lon = std::atan2(y, x);
        double p = std::sqrt(x*x + y*y);
        lat = std::atan2(z, p);
        alt = p / std::cos(lat) - getRadius();
    }
    
    // --- ComponentBase 生命周期 ---
    
    void update(double dt) override {
        (void)dt;
        // 地球模型通常不需要更新
    }
};

// 自动注册到工厂
GNC_REGISTER_COMPONENT(Wgs84Earth, interfaces::IEarthModel)

} // namespace gnc::components
