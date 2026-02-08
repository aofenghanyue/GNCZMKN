/**
 * @file i_disturbance.hpp
 * @brief 扰动接口
 */
#pragma once

#include "gnc/interfaces/data_types.hpp"

namespace gnc::interfaces {

/**
 * @brief 扰动源接口
 * 
 * 扰动可以叠加，动力学模块可以持有多个扰动源
 */
class IDisturbance {
public:
    virtual ~IDisturbance() = default;
    
    /// 获取当前扰动数据
    virtual DisturbanceData getDisturbance() const = 0;
    
    /// 获取扰动名称（用于调试/日志）
    virtual const std::string& getName() const = 0;
    
    /// 扰动是否激活
    virtual bool isEnabled() const = 0;
    
    /// 设置扰动开关
    virtual void setEnabled(bool enabled) = 0;
};

} // namespace gnc::interfaces
