/**
 * @file i_controller.hpp
 * @brief 控制器接口
 */
#pragma once

#include "gnc/common/math_types.hpp"
#include "gnc/interfaces/data_types.hpp"

namespace gnc::interfaces {

/// 控制指令数据包
struct ControlCommand {
    Vector3d torque_cmd;        ///< 力矩指令 (Nm)
    Vector3d force_cmd;         ///< 力指令 (N)
    double timestamp = 0.0;
};

/**
 * @brief 控制算法接口
 */
class IController {
public:
    virtual ~IController() = default;
    
    /// 获取当前控制指令
    virtual const ControlCommand& getControlCommand() const = 0;
    
    /// 获取执行器指令
    virtual const ActuatorCommand& getActuatorCommand() const = 0;
    
    /// 控制器是否激活
    virtual bool isActive() const = 0;
};

} // namespace gnc::interfaces
