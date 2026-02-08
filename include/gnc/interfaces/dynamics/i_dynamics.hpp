/**
 * @file i_dynamics.hpp
 * @brief 动力学接口
 */
#pragma once

#include "gnc/interfaces/data_types.hpp"

namespace gnc::interfaces {

/**
 * @brief 动力学/运动学方程接口
 */
class IDynamics {
public:
    virtual ~IDynamics() = default;
    
    /// 获取当前飞行器状态
    virtual const VehicleState& getVehicleState() const = 0;
    
    /// 设置外力 (惯性系, N)
    virtual void setExternalForce(const Vector3d& force) = 0;
    
    /// 设置外力矩 (机体系, Nm)
    virtual void setExternalTorque(const Vector3d& torque) = 0;
    
    /// 重置到初始状态
    virtual void reset() = 0;
};

} // namespace gnc::interfaces
