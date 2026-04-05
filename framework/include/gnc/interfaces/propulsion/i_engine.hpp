/**
 * @file i_engine.hpp
 * @brief 发动机接口
 */
#pragma once

#include "gnc/common/math_types.hpp"

namespace gnc::interfaces {

/**
 * @brief 发动机模型接口
 */
class IEngine {
public:
    virtual ~IEngine() = default;
    
    /// 设置油门指令 (0.0 - 1.0)
    virtual void setThrottle(double throttle) = 0;
    
    /// 获取当前推力 (N)
    virtual double getThrust() const = 0;
    
    /// 获取推力方向 (机体系)
    virtual Vector3d getThrustDirection() const = 0;
    
    /// 获取推力作用点 (机体系, m)
    virtual Vector3d getThrustPosition() const = 0;
    
    /// 获取燃料消耗率 (kg/s)
    virtual double getFuelConsumptionRate() const = 0;
    
    /// 发动机是否正在运行
    virtual bool isRunning() const = 0;
};

} // namespace gnc::interfaces
