/**
 * @file i_navigation.hpp
 * @brief 导航接口
 */
#pragma once

#include "gnc/common/math_types.hpp"

namespace gnc::interfaces {

/// 导航状态数据包
struct NavState {
    Vector3d position;          ///< 位置 (m)
    Vector3d velocity;          ///< 速度 (m/s)
    Quaterniond attitude;       ///< 姿态四元数
    Vector3d angular_velocity;  ///< 机体角速度 (rad/s)
    double timestamp = 0.0;
};

/**
 * @brief 导航解算接口
 */
class INavigation {
public:
    virtual ~INavigation() = default;
    
    /// 获取当前导航状态
    virtual const NavState& getNavState() const = 0;
    
    /// 获取导航时间戳
    virtual double getTimestamp() const = 0;
    
    /// 导航是否收敛/有效
    virtual bool isValid() const = 0;
};

} // namespace gnc::interfaces
