/**
 * @file i_mass_property.hpp
 * @brief 质量特性接口
 */
#pragma once

#include "gnc/common/math_types.hpp"

namespace gnc::interfaces {

/**
 * @brief 质量/质心/惯量 接口
 */
class IMassProperty {
public:
    virtual ~IMassProperty() = default;
    
    /// 获取当前质量 (kg)
    virtual double getMass() const = 0;
    
    /// 获取质心位置 (机体系, m)
    virtual Vector3d getCenterOfMass() const = 0;
    
    /// 获取惯性矩阵 (kg·m²)
    virtual Matrix3d getInertiaMatrix() const = 0;
    
    /// 更新质量特性（如燃料消耗）
    virtual void updateMassProperties(double dt) = 0;
};

} // namespace gnc::interfaces
