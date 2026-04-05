/**
 * @file i_actuator.hpp
 * @brief 执行机构接口
 */
#pragma once

#include "gnc/common/math_types.hpp"

namespace gnc::interfaces {

/**
 * @brief 执行机构接口（舵面、电机等）
 */
class IActuator {
public:
    virtual ~IActuator() = default;
    
    /// 设置指令 (具体含义由实现决定：角度/转速/推力等)
    virtual void setCommand(double command) = 0;
    
    /// 获取当前输出
    virtual double getOutput() const = 0;
    
    /// 获取产生的力 (机体系, N)
    virtual Vector3d getForce() const = 0;
    
    /// 获取产生的力矩 (机体系, Nm)
    virtual Vector3d getTorque() const = 0;
    
    /// 获取执行器名称
    virtual const std::string& getName() const = 0;
};

} // namespace gnc::interfaces
