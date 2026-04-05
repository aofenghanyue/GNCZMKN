/**
 * @file i_navigation.hpp
 * @brief 导航接口
 */
#pragma once

#include "gnc/interfaces/data_types.hpp"

namespace gnc::interfaces {

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
