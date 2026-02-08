/**
 * @file i_aerodynamics.hpp
 * @brief 气动模型接口
 */
#pragma once

#include "gnc/common/math_types.hpp"

namespace gnc::interfaces {

/**
 * @brief 气动系数/力矩计算接口
 */
class IAerodynamics {
public:
    virtual ~IAerodynamics() = default;
    
    /// 计算气动力 (机体系, N)
    virtual Vector3d computeAeroForce(double alpha, double beta, 
                                       double mach, double dynamic_pressure) const = 0;
    
    /// 计算气动力矩 (机体系, Nm)
    virtual Vector3d computeAeroMoment(double alpha, double beta,
                                        double mach, double dynamic_pressure) const = 0;
    
    /// 获取参考面积 (m²)
    virtual double getReferenceArea() const = 0;
    
    /// 获取参考长度 (m)
    virtual double getReferenceLength() const = 0;
};

} // namespace gnc::interfaces
