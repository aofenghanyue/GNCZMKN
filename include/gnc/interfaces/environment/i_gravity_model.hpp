/**
 * @file i_gravity_model.hpp
 * @brief 重力场模型接口
 */
#pragma once

#include "gnc/common/math_types.hpp"

namespace gnc::interfaces {

/**
 * @brief 重力场模型接口
 */
class IGravityModel {
public:
    virtual ~IGravityModel() = default;
    
    /// 获取指定位置的重力加速度矢量 (m/s²)
    virtual Vector3d getGravity(const Vector3d& position) const = 0;
    
    /// 获取海平面标准重力加速度 (m/s²)
    virtual double getSeaLevelGravity() const = 0;
};

} // namespace gnc::interfaces
